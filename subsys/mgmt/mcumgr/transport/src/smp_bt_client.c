/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Jeff Welder
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Bluetooth GATT client transport for the MCUmgr SMP client.
 *
 * The mirror image of the SMP Bluetooth server transport: that one is the GATT server
 * side of the SMP service, this one is the GATT client side. It discovers the SMP
 * service on a connected peer, subscribes to its notifications, writes every outbound
 * SMP packet out as a stream of ATT_MTU-3 sized Write Without Response writes, and feeds
 * the notifications that come back through the shared packet reassembly context into the
 * SMP core, which routes responses to the SMP client.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/mgmt/mcumgr/mgmt/handlers.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt_defines.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zephyr/mgmt/mcumgr/transport/smp.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt_client.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/clock.h>
#include <zephyr/sys/util.h>

#include <mgmt/mcumgr/transport/smp_internal.h>
#include <mgmt/mcumgr/transport/smp_reassembly.h>

LOG_MODULE_DECLARE(mcumgr_smp, CONFIG_MCUMGR_TRANSPORT_LOG_LEVEL);

/* Bytes an ATT Write Command spends on its own opcode and handle. */
#define SMP_BT_CLIENT_ATT_OVERHEAD 3U

#define SMP_BT_CLIENT_TX_TIMEOUT K_MSEC(CONFIG_MCUMGR_TRANSPORT_BT_CLIENT_TX_CREDIT_TIMEOUT)

/* The SMP service as every Zephyr SMP Bluetooth server publishes it. Taken from the
 * server transport's public header so that the two can never drift apart.
 */
static const struct bt_uuid_128 smp_bt_client_svc_uuid = BT_UUID_INIT_128(SMP_BT_SVC_UUID_VAL);
static const struct bt_uuid_128 smp_bt_client_chr_uuid = BT_UUID_INIT_128(SMP_BT_CHR_UUID_VAL);
static const struct bt_uuid_16 smp_bt_client_ccc_uuid = BT_UUID_INIT_16(BT_UUID_GATT_CCC_VAL);

enum smp_bt_client_link_state {
	/* No target: nothing is transmitted and inbound fragments are discarded. */
	SMP_BT_CLIENT_IDLE = 0,
	/* Discovery or the subscription is in progress; not yet able to carry traffic. */
	SMP_BT_CLIENT_ATTACHING,
	/* Attached: the transport carries SMP traffic to the target. */
	SMP_BT_CLIENT_READY,
};

static struct smp_transport smp_bt_client_transport;
static struct smp_client_transport_entry smp_bt_client_entry = {
	.smpt = &smp_bt_client_transport,
	.smpt_type = SMP_BLUETOOTH_CLIENT_TRANSPORT,
#ifdef CONFIG_MCUMGR_GRP_TRANSPORT_INFO_FUNCTIONS
	.name = "Bluetooth client",
#endif
};
static bool smp_bt_client_registered;

static atomic_t smp_bt_client_state = ATOMIC_INIT(SMP_BT_CLIENT_IDLE);

/*
 * The link the transport transmits on. Both members are written under the spin lock and
 * are read together through smp_bt_client_link_claim(), which hands back a counted
 * reference, so the MCUmgr work queue can neither dereference a connection the
 * disconnected callback has released nor pair one target's connection with another
 * target's value handle.
 */
static struct k_spinlock smp_bt_client_target_lock;
static struct bt_conn *smp_bt_client_target;
static uint16_t smp_bt_client_tx_handle;

/*
 * The connection whose client characteristic configuration write, if one is still in
 * flight, references the shared subscription parameters below. See those for why this is
 * tracked separately from whether the node is linked.
 */
static struct bt_conn *smp_bt_client_ccc_conn;

/*
 * Set while this module is inside bt_gatt_unsubscribe(). That call cancels a client
 * characteristic configuration write of its own that is still in flight, and the
 * cancellation runs this module's callbacks synchronously, from inside the call, before
 * the unsubscribe goes on to queue a replacement write with the very same parameters.
 * Those nested callbacks must not clear the ownership record, because the record has to
 * go on describing the replacement write once the call returns. Written only by the
 * thread making the unsubscribe, which is an application thread, and read only from the
 * callbacks.
 */
static bool smp_bt_client_ccc_owner_pinned;

/* Handles discovery found on the peer. Written only from the discovery callbacks. */
static uint16_t smp_bt_client_svc_end_handle;
static uint16_t smp_bt_client_value_handle;
static uint16_t smp_bt_client_ccc_handle;

/* One link, latched so that the transmit path reads a self consistent target. */
struct smp_bt_client_link {
	struct bt_conn *conn;
	uint16_t value_handle;
	uint32_t generation;
};

/*
 * A single discovery parameter block, walked from the primary service to the
 * characteristic to its client characteristic configuration descriptor.
 *
 * bt_gatt_discover() does not copy this struct: the Bluetooth host keeps writing to it,
 * advancing the start handle between ATT requests, until a terminal callback is
 * delivered. smp_bt_client_discovery_active therefore records that the host still owns
 * it, and is cleared only from a callback that ends the procedure. An attach that gives
 * up on its timeout leaves the block live, and the next attach must refuse rather than
 * rewrite a struct that the host is still reading.
 */
static struct bt_gatt_discover_params smp_bt_client_discover_params;
static atomic_t smp_bt_client_discovery_active = ATOMIC_INIT(0);

/*
 * A single subscription node that the host links into its per-connection subscription
 * list. Two rules keep reusing it safe:
 *
 *  - the notify and subscribe callbacks are installed once, at registration, and are
 *    never cleared. The host dereferences the notify callback without a NULL check, and
 *    a client characteristic configuration write response can be delivered after the
 *    node has already been unlinked.
 *  - every other field is rewritten only while smp_bt_client_sub_linked is false.
 *    Reinitialising a node that is still linked corrupts the host's list, so attach
 *    refuses rather than risk it.
 *
 * smp_bt_client_sub_linked is set when bt_gatt_subscribe() accepts the node, and cleared
 * on either of the two signals that mean the host has let go of it: a notify callback
 * with NULL data, or an unsubscribe that returned success. Both are needed. The NULL
 * notification is not universally emitted - an unsubscribe that unlinks the node and then
 * fails to write the descriptor never produces one - and an unsubscribe returning success
 * is not documented to unlink synchronously, so neither signal alone is sufficient.
 *
 * Being unlinked is not on its own enough to reuse the node. An unsubscribe removes the
 * node from the host's list and returns success while the descriptor write it issued is
 * still queued with these very parameters as its user data, and that write's response
 * later runs the subscribe callback and clears a flag bit in this struct. The connection
 * that write was issued on is therefore recorded in smp_bt_client_ccc_conn, attach
 * refuses while it is set, and the subscribe callback ignores any response that does not
 * belong to it. Recorded rather than inferred, because the parameters carry no per
 * request context of their own and the callback cannot otherwise tell which write it is
 * reporting on.
 */
static struct bt_gatt_subscribe_params smp_bt_client_sub_params;
static bool smp_bt_client_sub_linked;

/*
 * Handshake back to the blocked attach caller.
 *
 * An attach that gives up on its timeout can leave a client characteristic configuration
 * write in flight. Its late response must not be allowed to write handles or to give the
 * semaphore that the next attach is parked on, and it cannot simply be ignored on the
 * grounds that the node was unlinked, because the next attach links the very same node
 * again. Every attach therefore takes the next generation, and a callback that finds the
 * generation it captured is no longer current returns without touching anything.
 */
static K_SEM_DEFINE(smp_bt_client_attach_sem, 0, 1);
static int smp_bt_client_attach_result;
static atomic_t smp_bt_client_attach_generation = ATOMIC_INIT(0);
static uint32_t smp_bt_client_subscribe_generation;

/*
 * Write Without Response gets no ATT error back, so a dropped fragment surfaces only as
 * an SMP client timeout, which retries the whole packet.
 *
 * The wait for room to transmit must not be a sleep and poll loop: the transport's output
 * function runs on the shared MCUmgr work queue, which also drives this client's retry
 * timer and, on a device that is an SMP server as well, the server's response path.
 * Spinning there stalls both. Instead every fragment costs one credit from a counting
 * semaphore that the host's write completion callback returns.
 *
 * The credit count must not exceed the number of ATT transmit buffers. A credit is
 * returned only after the buffer it stands for has already gone back to that pool, so
 * more credits than buffers lets a fragment take a credit and then block inside the
 * Bluetooth host's buffer allocation instead, which takes no timeout at all when it is
 * called from a thread that is neither the system work queue nor the ATT response thread.
 * The MCUmgr work queue is neither. The build assertion below keeps that from being
 * configurable by accident.
 *
 * The two threads involved are always different - the completion callback is delivered
 * from the system work queue, the wait happens on the MCUmgr work queue - so the wait
 * cannot deadlock against its own completion.
 */
BUILD_ASSERT(CONFIG_MCUMGR_TRANSPORT_BT_CLIENT_TX_CREDITS <= CONFIG_BT_ATT_TX_COUNT,
	     "MCUMGR_TRANSPORT_BT_CLIENT_TX_CREDITS must not exceed BT_ATT_TX_COUNT, or a "
	     "fragment blocks in the Bluetooth host with no timeout");

static K_SEM_DEFINE(smp_bt_client_tx_sem, CONFIG_MCUMGR_TRANSPORT_BT_CLIENT_TX_CREDITS,
		    CONFIG_MCUMGR_TRANSPORT_BT_CLIENT_TX_CREDITS);

/*
 * Serialises the packet reassembly context, which has no locking of its own. The notify
 * callback runs on a Bluetooth host thread while detach runs on the caller's thread;
 * without this they can race a drop against an in-flight collect and free the packet
 * buffer out from under it. A mutex rather than a spin lock: collecting copies a whole
 * fragment, and every call site is thread context.
 */
static K_MUTEX_DEFINE(smp_bt_client_reassembly_lock);

static void smp_bt_client_tx_credits_replenish(void)
{
	/* k_sem_give() saturates at the initial count, so this restores full credit
	 * without disturbing a thread that is already waiting for one.
	 */
	for (int i = 0; i < CONFIG_MCUMGR_TRANSPORT_BT_CLIENT_TX_CREDITS; i++) {
		k_sem_give(&smp_bt_client_tx_sem);
	}
}

static void smp_bt_client_tx_done(struct bt_conn *conn, void *user_data)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(user_data);

	k_sem_give(&smp_bt_client_tx_sem);
}

/*
 * Hands back the current link with a counted reference on its connection, or false when
 * there is no target to transmit to. Connection and value handle are read under one lock
 * so that they always describe the same attach.
 */
static bool smp_bt_client_link_claim(struct smp_bt_client_link *link)
{
	bool claimed = false;
	k_spinlock_key_t key = k_spin_lock(&smp_bt_client_target_lock);

	if (smp_bt_client_target != NULL && smp_bt_client_tx_handle != 0U) {
		link->conn = bt_conn_ref(smp_bt_client_target);
		link->value_handle = smp_bt_client_tx_handle;
		link->generation = (uint32_t)atomic_get(&smp_bt_client_attach_generation);
		claimed = true;
	}

	k_spin_unlock(&smp_bt_client_target_lock, key);

	return claimed;
}

/*
 * Reports whether the link a transmit latched is still the one the transport is attached
 * to. The generation is part of the test because a detach followed by an attach to a
 * different peer can restore the ready state while an older packet is still being written
 * out fragment by fragment.
 */
static bool smp_bt_client_link_current(const struct smp_bt_client_link *link)
{
	return atomic_get(&smp_bt_client_state) == SMP_BT_CLIENT_READY &&
	       (uint32_t)atomic_get(&smp_bt_client_attach_generation) == link->generation;
}

/* The target connection alone, for teardown paths that run before or after the transmit
 * handle is published.
 */
static struct bt_conn *smp_bt_client_conn_claim(void)
{
	struct bt_conn *conn = NULL;
	k_spinlock_key_t key = k_spin_lock(&smp_bt_client_target_lock);

	if (smp_bt_client_target != NULL) {
		conn = bt_conn_ref(smp_bt_client_target);
	}

	k_spin_unlock(&smp_bt_client_target_lock, key);

	return conn;
}

/* Publishes the value handle the transmit path uses, unless the target is already gone. */
static bool smp_bt_client_tx_handle_install(uint16_t handle)
{
	bool installed = false;
	k_spinlock_key_t key = k_spin_lock(&smp_bt_client_target_lock);

	if (smp_bt_client_target != NULL) {
		smp_bt_client_tx_handle = handle;
		installed = true;
	}

	k_spin_unlock(&smp_bt_client_target_lock, key);

	return installed;
}

static void smp_bt_client_target_clear(void)
{
	struct bt_conn *conn;
	k_spinlock_key_t key = k_spin_lock(&smp_bt_client_target_lock);

	conn = smp_bt_client_target;
	smp_bt_client_target = NULL;
	smp_bt_client_tx_handle = 0U;
	k_spin_unlock(&smp_bt_client_target_lock, key);

	if (conn != NULL) {
		bt_conn_unref(conn);
	}
}

/* Records that a client characteristic configuration write may still be in flight. */
static void smp_bt_client_ccc_owner_set(struct bt_conn *conn)
{
	k_spinlock_key_t key = k_spin_lock(&smp_bt_client_target_lock);

	smp_bt_client_ccc_conn = conn;
	k_spin_unlock(&smp_bt_client_target_lock, key);
}

/*
 * Clears that record if it names @p conn, and reports whether it did. A false return in a
 * callback means the response belongs to a write this module no longer owns.
 */
static bool smp_bt_client_ccc_owner_release(struct bt_conn *conn)
{
	bool owned;
	k_spinlock_key_t key = k_spin_lock(&smp_bt_client_target_lock);

	owned = (smp_bt_client_ccc_conn != NULL && smp_bt_client_ccc_conn == conn);
	if (owned) {
		smp_bt_client_ccc_conn = NULL;
	}

	k_spin_unlock(&smp_bt_client_target_lock, key);

	return owned;
}

/*
 * Unsubscribes while holding the ownership record across the call. See
 * smp_bt_client_ccc_owner_pinned. The record is left set on success, because the call
 * leaves its own descriptor write in flight, and dropped by the caller on failure.
 */
static int smp_bt_client_unsubscribe(struct bt_conn *conn)
{
	int rc;

	smp_bt_client_ccc_owner_pinned = true;
	rc = bt_gatt_unsubscribe(conn, &smp_bt_client_sub_params);
	smp_bt_client_ccc_owner_pinned = false;

	if (rc == 0 && !atomic_test_bit(smp_bt_client_sub_params.flags,
					BT_GATT_SUBSCRIBE_FLAG_WRITE_PENDING)) {
		/* Another subscription still covers the handle, so the host unlinked the
		 * node without a descriptor write, or the write has already been answered.
		 * Either way nothing is in flight for the owner record to guard.
		 */
		(void)smp_bt_client_ccc_owner_release(conn);
	}

	return rc;
}

static bool smp_bt_client_ccc_owner_busy(void)
{
	bool busy;
	k_spinlock_key_t key = k_spin_lock(&smp_bt_client_target_lock);

	busy = (smp_bt_client_ccc_conn != NULL);
	k_spin_unlock(&smp_bt_client_target_lock, key);

	return busy;
}

static void smp_bt_client_reassembly_clear(void)
{
	k_mutex_lock(&smp_bt_client_reassembly_lock, K_FOREVER);

	if (smp_reassembly_expected(&smp_bt_client_transport) >= 0) {
		(void)smp_reassembly_drop(&smp_bt_client_transport);
	}

	k_mutex_unlock(&smp_bt_client_reassembly_lock);
}

/*
 * Waits for one transmit credit.
 *
 * A packet that has not started yet may be dropped when no credit turns up, because
 * nothing has been committed to the peer. Once a fragment is on the wire that is no
 * longer true: the stream carries no framing of its own, so a peer left holding half a
 * packet appends whatever arrives next to it and stays misframed. A started packet
 * therefore keeps waiting, and is only given up on when the link it was latched from
 * stops being the transport's target - which the disconnected callback guarantees by
 * handing every credit back.
 */
static bool smp_bt_client_tx_credit_take(const struct smp_bt_client_link *link, bool started)
{
	while (k_sem_take(&smp_bt_client_tx_sem, SMP_BT_CLIENT_TX_TIMEOUT) != 0) {
		if (!started) {
			LOG_WRN("SMP client transmit credit starved after %d ms",
				CONFIG_MCUMGR_TRANSPORT_BT_CLIENT_TX_CREDIT_TIMEOUT);
			return false;
		}

		if (!smp_bt_client_link_current(link)) {
			LOG_WRN("SMP client target went away with a packet part written");
			return false;
		}
	}

	return true;
}

/*
 * Writes one SMP packet out as a stream of ATT_MTU-3 sized fragments. Exactly one packet
 * per stream, never two packets coalesced into a single write, because the peer's
 * reassembly context keys off nothing but arrival order.
 *
 * The link is latched by the caller and the value handle comes from it rather than from
 * module state, so a detach and re-attach part way through cannot redirect the remaining
 * fragments of this packet to another peer's handle.
 */
static int smp_bt_client_tx_frags(const struct smp_bt_client_link *link, struct net_buf *nb)
{
	uint16_t mtu = bt_gatt_get_mtu(link->conn);
	uint16_t frag_max;
	uint16_t off = 0;

	if (mtu <= SMP_BT_CLIENT_ATT_OVERHEAD) {
		return MGMT_ERR_EUNKNOWN;
	}

	frag_max = mtu - SMP_BT_CLIENT_ATT_OVERHEAD;

	while (off < nb->len) {
		uint16_t len = MIN(frag_max, (uint16_t)(nb->len - off));
		int err;

		if (!smp_bt_client_tx_credit_take(link, off != 0U)) {
			return MGMT_ERR_EUNKNOWN;
		}

		if (!smp_bt_client_link_current(link)) {
			/* Detached or disconnected while this packet was being
			 * written out. There is nothing useful left to send.
			 */
			k_sem_give(&smp_bt_client_tx_sem);
			return MGMT_ERR_ENOENT;
		}

		err = bt_gatt_write_without_response_cb(link->conn, link->value_handle,
							&nb->data[off], len, false,
							smp_bt_client_tx_done, NULL);
		if (err != 0) {
			/* The completion callback only runs for a write the host
			 * accepted, so return the credit by hand. A failure part way
			 * through leaves the peer holding an incomplete packet, which
			 * it discards once the retransmission overruns it.
			 */
			k_sem_give(&smp_bt_client_tx_sem);
			LOG_WRN("SMP client fragment write failed (err %d, %u of %u bytes "
				"sent), dropping packet",
				err, off, nb->len);

			return (err == -ENOMEM) ? MGMT_ERR_ENOMEM : MGMT_ERR_EUNKNOWN;
		}

		off += len;
	}

	return MGMT_ERR_EOK;
}

/*
 * Transport transmit, called from the MCUmgr work queue for every SMP client request and
 * for every retransmission of one.
 *
 * The SMP client keeps its own reference to the buffer so that it can hand back the very
 * same one on a retry, so this must treat it as read only: no pull, no length edit, no
 * user data teardown, and exactly one reference released here.
 */
static int smp_bt_client_tx_pkt(struct net_buf *nb)
{
	struct smp_bt_client_link link;
	int rc = MGMT_ERR_ENOENT;

	/* Clearing the ready state does not drain a work item that has already been
	 * dequeued, so a packet can still arrive here after a detach or a disconnect.
	 * Refusing to transmit unless the transport is ready is what makes that safe,
	 * and the fragment loop keeps re-testing it because writing a whole packet out
	 * is not instantaneous.
	 */
	if (atomic_get(&smp_bt_client_state) == SMP_BT_CLIENT_READY &&
	    smp_bt_client_link_claim(&link)) {
		if (smp_bt_client_link_current(&link)) {
			rc = smp_bt_client_tx_frags(&link, nb);
		}

		bt_conn_unref(link.conn);
	}

	smp_packet_free(nb);

	return rc;
}

/*
 * Calculates the maximum fragment size for the current target. The buffer carries no
 * connection context, because this transport has exactly one target at a time.
 */
static uint16_t smp_bt_client_get_mtu(const struct net_buf *nb)
{
	struct smp_bt_client_link link;
	uint16_t mtu;

	ARG_UNUSED(nb);

	if (!smp_bt_client_link_claim(&link)) {
		return 0;
	}

	mtu = bt_gatt_get_mtu(link.conn);
	bt_conn_unref(link.conn);

	if (mtu <= SMP_BT_CLIENT_ATT_OVERHEAD) {
		return 0;
	}

	/* Account for the three byte ATT write header. */
	return mtu - SMP_BT_CLIENT_ATT_OVERHEAD;
}

/*
 * Transport receive. Fragments go through the shared reassembly context and a completed
 * packet is handed to the SMP core, which routes response operations to the SMP client.
 */
static uint8_t smp_bt_client_notify_cb(struct bt_conn *conn,
				       struct bt_gatt_subscribe_params *params, const void *data,
				       uint16_t length)
{
	int ret;

	ARG_UNUSED(params);

	if (data == NULL) {
		/* The host's signal that this node is no longer in the subscription
		 * list, and that it has finished with the descriptor write that
		 * removed it. Only after this may attach rewrite the node.
		 *
		 * The subscribed value handle is deliberately left alone: by the time
		 * this is delivered the next attach may already have installed a new
		 * subscription on the same node, and clearing the handle would silently
		 * stop notifications from being routed to it.
		 */
		if (!smp_bt_client_ccc_owner_pinned) {
			(void)smp_bt_client_ccc_owner_release(conn);
		}

		smp_bt_client_sub_linked = false;
		return BT_GATT_ITER_STOP;
	}

	if (length == 0U) {
		return BT_GATT_ITER_CONTINUE;
	}

	k_mutex_lock(&smp_bt_client_reassembly_lock, K_FOREVER);

	/*
	 * The ready test belongs inside the lock. Detach and the disconnected callback
	 * both leave the ready state and then drop the reassembly context under this
	 * same mutex. A fragment that tested the state outside the lock could pass while
	 * the transport was still ready, block on the lock while the teardown ran, and
	 * then collect on the far side of it, creating a fresh partial context out of one
	 * orphan fragment. Nothing clears that afterwards, so the next attach would start
	 * with a half assembled packet and misframe its first real response.
	 *
	 * Holding the lock across the test closes that: the teardown either has not
	 * reached its drop, in which case it waits here, or has already left the ready
	 * state, in which case this returns without touching the context.
	 */
	if (atomic_get(&smp_bt_client_state) != SMP_BT_CLIENT_READY) {
		k_mutex_unlock(&smp_bt_client_reassembly_lock);
		return BT_GATT_ITER_CONTINUE;
	}

	/*
	 * Handed on as every transport hands on what it receives, so a request the peer
	 * sends is executed by whatever management groups this image serves. Restrict the
	 * groups or use the MCUmgr management hooks if that is not wanted.
	 */
	ret = smp_reassembly_collect(&smp_bt_client_transport, data, length);

	if (ret == -ENOMEM) {
		/* No buffer was allocated, so there is nothing to drop. */
		LOG_WRN("SMP client reassembly buffer exhausted");
	} else if (ret < 0) {
		LOG_WRN("SMP client reassembly failed (%d), dropping packet", ret);
		(void)smp_reassembly_drop(&smp_bt_client_transport);
	} else if (ret == 0) {
		/* No more bytes are expected for this packet. */
		(void)smp_reassembly_complete(&smp_bt_client_transport, false);
	}

	k_mutex_unlock(&smp_bt_client_reassembly_lock);

	/* Never stop iterating on real data: the host reads that as a request to
	 * unsubscribe and tears the subscription down.
	 */
	return BT_GATT_ITER_CONTINUE;
}

static void smp_bt_client_subscribe_cb(struct bt_conn *conn, uint8_t err,
				       struct bt_gatt_subscribe_params *params)
{
	ARG_UNUSED(params);

	if (smp_bt_client_ccc_owner_pinned) {
		/* The response to the descriptor write that this module's own
		 * bt_gatt_unsubscribe() call is cancelling from under us. It is not a
		 * verdict on anything, and the ownership record stays set for the
		 * replacement write that same call is about to queue.
		 */
		return;
	}

	if (!smp_bt_client_ccc_owner_release(conn)) {
		/* The response to a descriptor write this module no longer owns, such
		 * as the one an unsubscribe issued on a link that has since been
		 * replaced. The parameters carry no per request context, so the
		 * connection the write went out on is the only thing that identifies
		 * it, and acting on it would report another link's verdict to whoever
		 * is attaching now.
		 */
		return;
	}

	if (smp_bt_client_subscribe_generation !=
	    (uint32_t)atomic_get(&smp_bt_client_attach_generation)) {
		/* A late response for an attach that has already given up. */
		return;
	}

	if (err != 0) {
		/* The ATT error is the whole diagnosis: insufficient authentication or
		 * encryption means the peer's SMP characteristic demands a secured link.
		 */
		LOG_ERR("SMP client subscribe rejected (ATT error 0x%02x)", err);
	}

	smp_bt_client_attach_result = (err == 0) ? 0 : -EIO;
	k_sem_give(&smp_bt_client_attach_sem);
}

static void smp_bt_client_discovery_finish(int result)
{
	smp_bt_client_attach_result = result;

	/*
	 * Order matters. Releasing the waiter before dropping the in-flight flag
	 * guarantees that a give belonging to an abandoned attach lands before any
	 * later attach can start, and every attach resets the semaphore before it starts
	 * a discovery, so such a give is always consumed by that reset.
	 */
	k_sem_give(&smp_bt_client_attach_sem);
	atomic_clear(&smp_bt_client_discovery_active);
}

static uint8_t smp_bt_client_ccc_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				    struct bt_gatt_discover_params *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	if (attr == NULL) {
		LOG_ERR("SMP characteristic has no client characteristic configuration");
		smp_bt_client_discovery_finish(-ENOTSUP);
		return BT_GATT_ITER_STOP;
	}

	/* Descriptor discovery reports whatever handle the peer put in its response,
	 * without the sanity check that characteristic discovery applies, so a peer can
	 * hand back a handle that is not one. Subscribing asserts on a zero handle, so
	 * reject anything outside the range this leg asked about instead.
	 */
	if (attr->handle <= smp_bt_client_value_handle ||
	    attr->handle > smp_bt_client_svc_end_handle) {
		LOG_ERR("Peer reported an out of range descriptor handle 0x%04x", attr->handle);
		smp_bt_client_discovery_finish(-ENOTSUP);
		return BT_GATT_ITER_STOP;
	}

	smp_bt_client_ccc_handle = attr->handle;
	smp_bt_client_discovery_finish(0);

	return BT_GATT_ITER_STOP;
}

static uint8_t smp_bt_client_chrc_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				     struct bt_gatt_discover_params *params)
{
	const uint8_t required = BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_NOTIFY;
	const struct bt_gatt_chrc *chrc;
	int err;

	if (attr == NULL) {
		LOG_ERR("Peer's SMP service has no SMP characteristic");
		smp_bt_client_discovery_finish(-ENOTSUP);
		return BT_GATT_ITER_STOP;
	}

	/* The discovery filtered on the characteristic UUID already; the attribute
	 * handed back is the characteristic declaration, so the value handle has to be
	 * read out of it rather than assumed to follow the declaration.
	 */
	chrc = attr->user_data;

	if ((chrc->properties & required) != required) {
		LOG_ERR("SMP characteristic properties 0x%02x cannot carry SMP", chrc->properties);
		smp_bt_client_discovery_finish(-ENOTSUP);
		return BT_GATT_ITER_STOP;
	}

	smp_bt_client_value_handle = chrc->value_handle;

	if (smp_bt_client_value_handle == 0U ||
	    smp_bt_client_value_handle >= smp_bt_client_svc_end_handle) {
		LOG_ERR("SMP characteristic leaves no room for a descriptor");
		smp_bt_client_discovery_finish(-ENOTSUP);
		return BT_GATT_ITER_STOP;
	}

	/* The client characteristic configuration descriptor is searched for rather than
	 * assumed to sit directly after the value attribute, because a server is free to
	 * place other descriptors first. The search runs to the end of the service and
	 * the host reports the first matching descriptor it meets, which is this
	 * characteristic's own: the SMP service declares a single characteristic, and a
	 * characteristic that can notify is required to carry a configuration descriptor
	 * (Core Specification Vol. 3, Part G, 3.3.3.3). Only a peer that breaks that
	 * rule, by declaring a notifiable characteristic without one, could hand back a
	 * later characteristic's descriptor instead.
	 */
	params->uuid = &smp_bt_client_ccc_uuid.uuid;
	params->func = smp_bt_client_ccc_cb;
	params->start_handle = smp_bt_client_value_handle + 1U;
	params->end_handle = smp_bt_client_svc_end_handle;
	params->type = BT_GATT_DISCOVER_DESCRIPTOR;

	err = bt_gatt_discover(conn, params);
	if (err != 0) {
		/* No terminal callback is delivered for a request the host never
		 * queued, so this leg has to end the procedure itself.
		 */
		LOG_ERR("SMP descriptor discovery could not be started (err %d)", err);
		smp_bt_client_discovery_finish(err);
	}

	return BT_GATT_ITER_STOP;
}

static uint8_t smp_bt_client_svc_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				    struct bt_gatt_discover_params *params)
{
	const struct bt_gatt_service_val *svc;
	int err;

	if (attr == NULL) {
		LOG_WRN("Peer does not expose an SMP service");
		smp_bt_client_discovery_finish(-ENOTSUP);
		return BT_GATT_ITER_STOP;
	}

	/* The attribute handed back is the primary service declaration, so its own UUID
	 * is the primary service UUID rather than the SMP one. What is wanted from it is
	 * the handle range the rest of the discovery has to stay inside.
	 */
	svc = attr->user_data;
	smp_bt_client_svc_end_handle = svc->end_handle;

	if (attr->handle >= smp_bt_client_svc_end_handle) {
		LOG_ERR("SMP service declares an empty handle range");
		smp_bt_client_discovery_finish(-ENOTSUP);
		return BT_GATT_ITER_STOP;
	}

	params->uuid = &smp_bt_client_chr_uuid.uuid;
	params->func = smp_bt_client_chrc_cb;
	params->start_handle = attr->handle + 1U;
	params->end_handle = smp_bt_client_svc_end_handle;
	params->type = BT_GATT_DISCOVER_CHARACTERISTIC;

	err = bt_gatt_discover(conn, params);
	if (err != 0) {
		LOG_ERR("SMP characteristic discovery could not be started (err %d)", err);
		smp_bt_client_discovery_finish(err);
	}

	return BT_GATT_ITER_STOP;
}

/* A released connection must never reach the transmit path, and the host drops the
 * subscription on disconnect anyway, so the target is torn down here rather than waiting
 * for the application to notice.
 */
static void smp_bt_client_disconnected(struct bt_conn *conn, uint8_t reason)
{
	k_spinlock_key_t key;
	bool ours;

	ARG_UNUSED(reason);

	/* Done before the target test and for every connection: a descriptor write that
	 * fails after the node has already been unlinked produces no callback at all, so
	 * the link going away is the only thing left that can end the host's claim on the
	 * subscription parameters. Without this the next attach would refuse for good.
	 */
	(void)smp_bt_client_ccc_owner_release(conn);

	key = k_spin_lock(&smp_bt_client_target_lock);
	ours = (conn == smp_bt_client_target);
	k_spin_unlock(&smp_bt_client_target_lock, key);

	if (!ours) {
		return;
	}

	LOG_DBG("SMP client target disconnected, detaching transport");

	/* The linked flag is deliberately not cleared here: the host clears it through
	 * the notify callback with NULL data when it sweeps this connection's
	 * subscriptions. Clearing it early would let the next attach rewrite a node that
	 * is still in the host's list.
	 */
	atomic_set(&smp_bt_client_state, SMP_BT_CLIENT_IDLE);

	smp_bt_client_reassembly_clear();

	/* A fragment blocked on a transmit credit will never get one back from this
	 * link: the host does not run the write completion callback once the bearer is
	 * gone. Hand the credits back so a transmit that can no longer complete cannot
	 * wedge the MCUmgr work queue.
	 */
	smp_bt_client_tx_credits_replenish();

	smp_rx_clear(&smp_bt_client_transport);
	smp_bt_client_target_clear();

	/* Release an attach still waiting on discovery or on the descriptor write. */
	smp_bt_client_attach_result = -ENOTCONN;
	k_sem_give(&smp_bt_client_attach_sem);
}

BT_CONN_CB_DEFINE(smp_bt_client_conn_callbacks) = {
	.disconnected = smp_bt_client_disconnected,
};

int smp_bt_client_attach(struct bt_conn *conn, k_timeout_t timeout)
{
	/* One deadline for both waits. Computed rather than decomposed, because a
	 * k_timeout_t is opaque: an absolute timeout does not hold a duration.
	 */
	const k_timepoint_t deadline = sys_timepoint_calc(timeout);
	uint32_t generation;
	k_spinlock_key_t key;
	int rc;

	if (conn == NULL) {
		return -EINVAL;
	}

	if (!smp_bt_client_registered) {
		return -ENODEV;
	}

	if (!atomic_cas(&smp_bt_client_state, SMP_BT_CLIENT_IDLE, SMP_BT_CLIENT_ATTACHING)) {
		return -EBUSY;
	}

	if (smp_bt_client_sub_linked) {
		/* A previous teardown never completed, so the node is still in the
		 * host's subscription list. Rewriting a linked node corrupts that list,
		 * so fail closed and let the caller retry once the host has released it.
		 */
		LOG_WRN("SMP client subscription still linked, refusing attach");
		atomic_set(&smp_bt_client_state, SMP_BT_CLIENT_IDLE);
		return -EBUSY;
	}

	if (smp_bt_client_ccc_owner_busy()) {
		/* A descriptor write from an earlier attach or detach still carries the
		 * subscription parameters as its user data. Rewriting them now would
		 * race the host and let that write's response be read as this attach's
		 * verdict, so fail closed until the host is finished with them.
		 */
		LOG_WRN("SMP client descriptor write still in flight, refusing attach");
		atomic_set(&smp_bt_client_state, SMP_BT_CLIENT_IDLE);
		return -EBUSY;
	}

	if (!atomic_cas(&smp_bt_client_discovery_active, 0, 1)) {
		/* The host still owns the discovery parameters from a previous attach. */
		LOG_WRN("SMP client discovery still in flight, refusing attach");
		atomic_set(&smp_bt_client_state, SMP_BT_CLIENT_IDLE);
		return -EBUSY;
	}

	generation = (uint32_t)atomic_inc(&smp_bt_client_attach_generation) + 1U;

	k_sem_reset(&smp_bt_client_attach_sem);
	smp_bt_client_attach_result = -ETIMEDOUT;
	smp_bt_client_svc_end_handle = 0;
	smp_bt_client_value_handle = 0;
	smp_bt_client_ccc_handle = 0;

	/* Credits that the previous link's completion callbacks never returned died with
	 * that link, so every attach starts from the full count.
	 */
	smp_bt_client_tx_credits_replenish();

	key = k_spin_lock(&smp_bt_client_target_lock);
	smp_bt_client_target = bt_conn_ref(conn);
	k_spin_unlock(&smp_bt_client_target_lock, key);

	smp_bt_client_discover_params.uuid = &smp_bt_client_svc_uuid.uuid;
	smp_bt_client_discover_params.func = smp_bt_client_svc_cb;
	smp_bt_client_discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	smp_bt_client_discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	smp_bt_client_discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	rc = bt_gatt_discover(conn, &smp_bt_client_discover_params);
	if (rc != 0) {
		LOG_ERR("SMP service discovery could not be started (err %d)", rc);
		atomic_clear(&smp_bt_client_discovery_active);
		goto fail;
	}

	if (k_sem_take(&smp_bt_client_attach_sem, sys_timepoint_timeout(deadline)) != 0) {
		rc = -ETIMEDOUT;
		goto fail;
	}

	rc = smp_bt_client_attach_result;
	if (rc != 0) {
		goto fail;
	}

	/* Provably unlinked here, guarded on entry, so the node's own fields are free to
	 * rewrite. The callbacks are not touched: registration owns them for the lifetime
	 * of the module.
	 */
	smp_bt_client_sub_params.value_handle = smp_bt_client_value_handle;
	smp_bt_client_sub_params.ccc_handle = smp_bt_client_ccc_handle;
	smp_bt_client_sub_params.value = BT_GATT_CCC_NOTIFY;
	memset(smp_bt_client_sub_params.flags, 0, sizeof(smp_bt_client_sub_params.flags));
	atomic_set_bit(smp_bt_client_sub_params.flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);
#if defined(CONFIG_BT_SMP)
	/* Take notifications at whatever security level the link already has. The peer's
	 * own characteristic permissions decide what it requires, and a higher local
	 * minimum would silently discard notifications the peer agreed to send.
	 */
	smp_bt_client_sub_params.min_security = BT_SECURITY_L1;
#endif

	k_sem_reset(&smp_bt_client_attach_sem);
	smp_bt_client_attach_result = -ETIMEDOUT;
	smp_bt_client_subscribe_generation = generation;

	/*
	 * Marked linked before the call, not after it. The host can unlink the node and
	 * report that through the notify callback from another thread as soon as the
	 * subscription exists, and a flag set afterwards would overwrite that report and
	 * leave the node marked linked for good, refusing every later attach.
	 */
	smp_bt_client_sub_linked = true;
	smp_bt_client_ccc_owner_set(conn);

	rc = bt_gatt_subscribe(conn, &smp_bt_client_sub_params);
	if (rc == -EALREADY) {
		/* Only returned when this node is already in the host's list, which the
		 * entry guard should have made impossible. Recover by trusting the host:
		 * it is linked, and there is no descriptor write left to wait for.
		 */
		(void)smp_bt_client_ccc_owner_release(conn);
		rc = 0;
	} else if (rc != 0) {
		/* The node was never linked and no write went out, so the host owes
		 * neither a removal report nor a write response.
		 */
		(void)smp_bt_client_ccc_owner_release(conn);
		smp_bt_client_sub_linked = false;
		LOG_ERR("SMP characteristic subscribe failed (err %d)", rc);
		goto fail;
	} else if (!atomic_test_bit(smp_bt_client_sub_params.flags,
				    BT_GATT_SUBSCRIBE_FLAG_WRITE_PENDING)) {
		/* The host linked the node without writing the descriptor, because another
		 * subscription on this connection already has notifications enabled for the
		 * handle, and it will deliver no response for a write it never sent. The
		 * flag also clears once a write has been answered, so the wait below is
		 * consulted without blocking before concluding that nothing was sent.
		 */
		if (k_sem_take(&smp_bt_client_attach_sem, K_NO_WAIT) != 0) {
			(void)smp_bt_client_ccc_owner_release(conn);
			smp_bt_client_attach_result = 0;
		}

		rc = smp_bt_client_attach_result;
		if (rc != 0) {
			goto fail;
		}
	} else {
		/* Wait for the descriptor write to land: a request sent before the peer
		 * has notifications enabled costs a full SMP client timeout cycle.
		 */
		if (k_sem_take(&smp_bt_client_attach_sem, sys_timepoint_timeout(deadline)) != 0) {
			rc = -ETIMEDOUT;
			goto fail;
		}

		rc = smp_bt_client_attach_result;
		if (rc != 0) {
			goto fail;
		}
	}

	/*
	 * Publishing the target and taking the ready state are both conditional on this
	 * attach still owning the transport. A disconnect that lands between the wait
	 * above and here has already released the target and returned the transport to
	 * idle, and an unconditional store would then leave it ready with nothing to
	 * transmit to and no way back short of a detach the application has no reason to
	 * call.
	 */
	if (!smp_bt_client_tx_handle_install(smp_bt_client_value_handle) ||
	    !atomic_cas(&smp_bt_client_state, SMP_BT_CLIENT_ATTACHING, SMP_BT_CLIENT_READY)) {
		rc = -ENOTCONN;
		goto fail;
	}

	LOG_INF("SMP client transport attached (value handle 0x%04x, MTU %u)",
		smp_bt_client_value_handle, bt_gatt_get_mtu(conn));

	return 0;

fail:
	/* Never leave a linked node behind for the next attach to trip over. */
	if (smp_bt_client_sub_linked) {
		int unsub;

		/* The unsubscribe issues a descriptor write of its own, which stays in
		 * flight with these parameters as its user data after the call returns.
		 */
		smp_bt_client_ccc_owner_set(conn);
		unsub = smp_bt_client_unsubscribe(conn);

		if (unsub == 0) {
			smp_bt_client_sub_linked = false;
		} else {
			/* The call left no descriptor write of its own behind, so
			 * nothing is owed on these parameters and the record has to be
			 * dropped whether or not the node came out of the host's list
			 * along the way. Dropping it unconditionally matters: a
			 * cancelled write can reach the notify callback with NULL data
			 * from inside the call, and a record left set with no write in
			 * flight refuses every later attach until the link drops.
			 */
			(void)smp_bt_client_ccc_owner_release(conn);
			LOG_WRN("SMP client unsubscribe after failed attach did not "
				"complete (err %d)",
				unsub);
		}
	}

	/* Only undo what this attach still owns. A disconnect may already have torn the
	 * target down, and the state it left behind may since have been taken by another
	 * attach, which must not have its own target cleared from under it. Whichever
	 * path returned the transport to idle cleared the target on the way.
	 */
	if ((uint32_t)atomic_get(&smp_bt_client_attach_generation) == generation) {
		(void)atomic_cas(&smp_bt_client_state, SMP_BT_CLIENT_ATTACHING, SMP_BT_CLIENT_IDLE);
		smp_bt_client_reassembly_clear();
		smp_bt_client_target_clear();
	}

	return rc;
}

void smp_bt_client_detach(void)
{
	struct bt_conn *conn;

	/* Leave the ready state first: the notify callback discards fragments once the
	 * transport is no longer ready, so nothing can be collecting while the context
	 * is torn down below.
	 */
	atomic_set(&smp_bt_client_state, SMP_BT_CLIENT_IDLE);

	conn = smp_bt_client_conn_claim();
	if (conn != NULL) {
		if (smp_bt_client_sub_linked) {
			/* The descriptor write the unsubscribe issues outlives the
			 * call, so record the connection it went out on before making
			 * it. Attach refuses until the host reports back on it.
			 */
			smp_bt_client_ccc_owner_set(conn);

			if (smp_bt_client_unsubscribe(conn) == 0) {
				smp_bt_client_sub_linked = false;
			} else {
				(void)smp_bt_client_ccc_owner_release(conn);
			}
		}

		bt_conn_unref(conn);
	}

	smp_bt_client_reassembly_clear();
	smp_rx_clear(&smp_bt_client_transport);
	smp_bt_client_target_clear();
}

bool smp_bt_client_is_attached(void)
{
	return atomic_get(&smp_bt_client_state) == SMP_BT_CLIENT_READY;
}

static void smp_bt_client_setup(void)
{
	int rc;

	smp_bt_client_transport.functions.output = smp_bt_client_tx_pkt;
	smp_bt_client_transport.functions.get_mtu = smp_bt_client_get_mtu;

	rc = smp_transport_init(&smp_bt_client_transport);
	if (rc != 0) {
		LOG_ERR("Bluetooth client SMP transport init failed (err %d)", rc);
		return;
	}

	/* Installed once and never cleared: the host dereferences the notify callback
	 * without a NULL check, including from a descriptor write response that is
	 * delivered after the node has already been unlinked.
	 */
	smp_bt_client_sub_params.notify = smp_bt_client_notify_cb;
	smp_bt_client_sub_params.subscribe = smp_bt_client_subscribe_cb;

	rc = smp_client_transport_register(&smp_bt_client_entry);
	if (rc != 0) {
		LOG_ERR("SMP Bluetooth client transport type already claimed (err %d)", rc);
		return;
	}

	smp_bt_client_registered = true;
}

MCUMGR_HANDLER_DEFINE(smp_bt_client, smp_bt_client_setup);
