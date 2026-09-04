/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Jeff Welder
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The Bluetooth client SMP transport driven against a scripted GATT peer.
 *
 * The sibling suite in tests/subsys/mgmt/mcumgr/transport_bt_client builds the real
 * Bluetooth host and covers what the transport does with no peer at all. This one goes
 * the other way: it leaves the host out (see src/gatt_stub.c) and answers the transport's
 * GATT calls itself, so the discovery chain, the subscription handshake, the client
 * characteristic configuration ownership state machine, the transmit credits and the
 * fragmentation loop can all be driven, including down the paths a cooperative peer never
 * takes.
 *
 * The stub is modelled on subsys/bluetooth/host/gatt.c and answers from a work queue, so
 * a callback never runs on the thread that asked for it. That is what makes the timeout
 * and late callback cases real rather than decorative.
 *
 * What this suite does NOT do is exercise the wire. No ATT PDU is encoded anywhere, so
 * ATT_MTU negotiation, the ordering and delivery of real Write Without Response traffic,
 * the peer's descriptor permissions and the Bluetooth host's transmit buffer pressure are
 * all out of its reach, and the host semantics the stub reproduces are asserted here
 * rather than proven. Only hardware, or a two node BabbleSim test on a Linux host, closes
 * that gap.
 *
 * The receive path is not faked below the GATT line: notifications go through the real
 * packet reassembly context into the real SMP core, which routes the response to a real
 * SMP client object.
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt_defines.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zephyr/mgmt/mcumgr/smp/smp_client.h>
#include <zephyr/mgmt/mcumgr/transport/smp.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt_client.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include <mgmt/mcumgr/transport/smp_internal.h>

#include "gatt_stub.h"

/* Generous next to the stub's 5 ms per procedure leg, tight enough to fail fast. */
#define ATTACH_TIMEOUT K_MSEC(1000)

/* Long enough for the MCUmgr work queue to drain a request or a response. */
#define SETTLE_MS 100

/* An ATT_MTU that makes every interesting packet fragment. */
#define SMALL_MTU  67U
#define SMALL_FRAG (SMALL_MTU - 3U)

/*
 * The size of the notifications a peer sends back. Twenty bytes is what an unnegotiated
 * ATT_MTU of 23 leaves, and is the smallest fragment the SMP Bluetooth server transport
 * is willing to use, so it is the realistic floor.
 */
#define RSP_FRAG 20U

static struct bt_conn peer;
static struct smp_client_object bt_client;

static uint32_t client_user_data;
static uint8_t rsp_payload[64];
static uint16_t rsp_payload_len;
static int rsp_count;
static int rsp_timeouts;
static int cmds_sent;
static void *rsp_user_data;

static struct k_work_delayable disconnect_work;

static struct smp_transport *transport(void)
{
	return smp_client_transport_get(SMP_BLUETOOTH_CLIENT_TRANSPORT);
}

static int response_cb(struct net_buf *nb, void *user_data)
{
	rsp_user_data = user_data;

	if (nb == NULL) {
		/* The SMP client gave up on the command. */
		rsp_timeouts++;
		return 0;
	}

	rsp_count++;
	rsp_payload_len = MIN(nb->len, sizeof(rsp_payload));
	memcpy(rsp_payload, nb->data, rsp_payload_len);

	return 0;
}

static void attach_ok(void)
{
	int rc = smp_bt_client_attach(&peer, ATTACH_TIMEOUT);

	zassert_equal(rc, 0, "attach failed (%d)", rc);
	zassert_true(smp_bt_client_is_attached(), "transport not ready after a successful attach");
}

/* Hands one whole SMP packet to the transport's transmit function. */
static int transmit(const void *data, uint16_t len)
{
	struct net_buf *nb = smp_packet_alloc();

	zassert_not_null(nb, "no packet buffer");
	net_buf_add_mem(nb, data, len);

	return transport()->functions.output(nb);
}

/* Sends a real SMP command through the SMP client, which runs the transport's transmit
 * path on the MCUmgr work queue, exactly as an MCUmgr client group would.
 */
static void send_command(void)
{
	uint8_t payload[100];
	struct net_buf *nb;
	int rc;

	/* Long enough that the transport has to fragment it at any sane ATT_MTU. */
	memset(payload, 0x41, sizeof(payload));
	payload[0] = 0xbf;
	payload[sizeof(payload) - 1] = 0xff;

	nb = smp_client_buf_allocation(&bt_client, MGMT_GROUP_ID_OS, 0, MGMT_OP_WRITE,
				       SMP_MCUMGR_VERSION_1);
	zassert_not_null(nb, "no SMP client buffer");
	net_buf_add_mem(nb, payload, sizeof(payload));

	/* One second, so that a command a test deliberately leaves outstanding gives its
	 * buffer back to the pool before the next test needs it.
	 */
	rc = smp_client_send_cmd(&bt_client, nb, response_cb, &client_user_data, 1);
	zassert_equal(rc, MGMT_ERR_EOK, "send failed (%d)", rc);
	cmds_sent++;

	k_sleep(K_MSEC(SETTLE_MS));
}

/*
 * Builds the response the peer would send back to the request the transport just wrote
 * out, reading the sequence number out of the bytes that actually went to the peer rather
 * than assuming what the SMP client picked.
 */
static uint16_t build_response(uint8_t *out, size_t out_size, const uint8_t *payload,
			       uint16_t payload_len, uint8_t seq_offset)
{
	struct smp_hdr hdr;

	zassert_true(gatt_stub_log.stream_len >= sizeof(hdr), "no request went out");
	zassert_true(out_size >= sizeof(hdr) + payload_len, "response buffer too small");

	memcpy(&hdr, gatt_stub_log.stream, sizeof(hdr));
	hdr.nh_op = MGMT_OP_WRITE_RSP;
	hdr.nh_len = sys_cpu_to_be16(payload_len);
	hdr.nh_seq += seq_offset;

	memcpy(out, &hdr, sizeof(hdr));
	memcpy(&out[sizeof(hdr)], payload, payload_len);

	return sizeof(hdr) + payload_len;
}

/* A response body long enough to span several notifications. */
static void fill_body(uint8_t *body, size_t len, uint8_t tag)
{
	memset(body, tag, len);
	body[0] = 0xbf;
	body[len - 1] = 0xff;
}

/* Delivers a packet as the stream of notifications a real peer would send. */
static void notify_fragmented(const uint8_t *data, uint16_t len, uint16_t frag)
{
	uint16_t off = 0;

	while (off < len) {
		uint16_t chunk = MIN(frag, (uint16_t)(len - off));

		gatt_stub_notify(&data[off], chunk);
		off += chunk;
	}
}

static void disconnect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	gatt_stub_disconnect(&peer);
}

ZTEST(mcumgr_transport_bt_client_gatt, test_attach_walks_the_three_discovery_legs)
{
	attach_ok();

	zassert_equal(gatt_stub_log.discover_calls, 3, "expected three discovery legs, saw %d",
		      gatt_stub_log.discover_calls);
	zassert_equal(gatt_stub_log.subscribe_calls, 1, "expected exactly one subscribe");
	zassert_true(gatt_stub_is_subscribed(), "the node was not left in the host's list");
	zassert_equal(gatt_stub_sub_params()->value_handle, gatt_stub_peer.value_handle,
		      "subscribed to the wrong value handle");
	zassert_equal(gatt_stub_sub_params()->ccc_handle, gatt_stub_peer.ccc_handle,
		      "wrote the wrong descriptor handle");
	zassert_equal(gatt_stub_sub_params()->value, BT_GATT_CCC_NOTIFY,
		      "did not ask the peer for notifications");
	zassert_equal(peer.ref, 1, "attach did not take exactly one connection reference");
	/* The transport advertises the ATT_MTU less the three byte write header. */
	zassert_equal(transport()->functions.get_mtu(NULL), 244, "wrong MTU reported");
}

ZTEST(mcumgr_transport_bt_client_gatt, test_attach_rejects_a_peer_without_an_smp_service)
{
	gatt_stub_peer.has_svc = false;

	zassert_equal(smp_bt_client_attach(&peer, ATTACH_TIMEOUT), -ENOTSUP,
		      "attached to a peer with no SMP service");
	zassert_equal(gatt_stub_log.discover_calls, 1, "kept discovering past a missing service");
	zassert_equal(peer.ref, 0, "a failed attach leaked a connection reference");
}

ZTEST(mcumgr_transport_bt_client_gatt, test_attach_rejects_an_empty_service_handle_range)
{
	gatt_stub_peer.svc_end_handle = gatt_stub_peer.svc_handle;

	zassert_equal(smp_bt_client_attach(&peer, ATTACH_TIMEOUT), -ENOTSUP,
		      "attached to a service that declares no attributes");
	zassert_equal(gatt_stub_log.discover_calls, 1, "kept discovering past an empty service");
	zassert_equal(peer.ref, 0, "a failed attach leaked a connection reference");
}

ZTEST(mcumgr_transport_bt_client_gatt, test_attach_rejects_a_characteristic_that_cannot_notify)
{
	gatt_stub_peer.properties = BT_GATT_CHRC_WRITE_WITHOUT_RESP;

	zassert_equal(smp_bt_client_attach(&peer, ATTACH_TIMEOUT), -ENOTSUP,
		      "attached to a characteristic that cannot notify");
	zassert_equal(gatt_stub_log.subscribe_calls, 0,
		      "subscribed to a characteristic that cannot notify");
	zassert_equal(peer.ref, 0, "a failed attach leaked a connection reference");
}

/*
 * The value handle comes out of the characteristic declaration, so a peer is free to put
 * it at the very end of the service, leaving nowhere for the descriptor to be.
 */
ZTEST(mcumgr_transport_bt_client_gatt, test_attach_rejects_a_value_handle_with_no_room_after_it)
{
	gatt_stub_peer.value_handle = gatt_stub_peer.svc_end_handle;

	zassert_equal(smp_bt_client_attach(&peer, ATTACH_TIMEOUT), -ENOTSUP,
		      "attached to a characteristic with no room for a descriptor");
	zassert_equal(gatt_stub_log.discover_calls, 2, "started a descriptor discovery anyway");
	zassert_equal(peer.ref, 0, "a failed attach leaked a connection reference");
}

ZTEST(mcumgr_transport_bt_client_gatt, test_attach_rejects_a_peer_without_a_descriptor)
{
	gatt_stub_peer.has_ccc = false;

	zassert_equal(smp_bt_client_attach(&peer, ATTACH_TIMEOUT), -ENOTSUP,
		      "attached to a characteristic with no configuration descriptor");
	zassert_equal(gatt_stub_log.subscribe_calls, 0, "subscribed without a descriptor handle");
	zassert_equal(peer.ref, 0, "a failed attach leaked a connection reference");
}

/*
 * Descriptor discovery hands back whatever handle the peer put in its response, with no
 * sanity check of its own, and subscribing with a handle of zero asserts inside the host.
 */
ZTEST(mcumgr_transport_bt_client_gatt, test_attach_rejects_an_out_of_range_descriptor_handle)
{
	gatt_stub_peer.ccc_handle = 0x0099;

	zassert_equal(smp_bt_client_attach(&peer, ATTACH_TIMEOUT), -ENOTSUP,
		      "attached with a descriptor handle outside the service");
	zassert_equal(gatt_stub_log.subscribe_calls, 0,
		      "subscribed to a handle outside the service");
	zassert_equal(peer.ref, 0, "a failed attach leaked a connection reference");
}

/*
 * A discovery the host refuses to start gets no callback at all, so the leg that issued
 * it has to end the procedure itself or the next attach is locked out for good.
 */
ZTEST(mcumgr_transport_bt_client_gatt, test_attach_recovers_from_a_discovery_that_never_started)
{
	gatt_stub_faults.discover_err = -ENOMEM;

	zassert_equal(smp_bt_client_attach(&peer, ATTACH_TIMEOUT), -ENOMEM,
		      "attach did not report the discovery failure");
	zassert_equal(peer.ref, 0, "a failed attach leaked a connection reference");

	gatt_stub_faults.discover_err = 0;
	attach_ok();
}

/*
 * Same, one leg further in: the host refuses the request the first leg's callback issues,
 * where the only thing that can end the procedure is that callback itself.
 */
ZTEST(mcumgr_transport_bt_client_gatt, test_attach_recovers_from_a_second_leg_that_never_started)
{
	gatt_stub_faults.discover_err = -ENOBUFS;
	gatt_stub_faults.discover_err_after = 1;

	zassert_equal(smp_bt_client_attach(&peer, ATTACH_TIMEOUT), -ENOBUFS,
		      "attach did not report the failure of the second discovery leg");
	zassert_equal(gatt_stub_log.discover_calls, 2, "the first leg was not answered");
	zassert_equal(peer.ref, 0, "a failed attach leaked a connection reference");

	gatt_stub_faults.discover_err = 0;
	attach_ok();
}

/*
 * The subscribe call itself failing means the node was never linked and no descriptor
 * write went out, so nothing is owed and the next attach must be able to proceed.
 */
ZTEST(mcumgr_transport_bt_client_gatt, test_attach_recovers_from_a_subscribe_that_never_started)
{
	gatt_stub_faults.subscribe_err = -ENOMEM;

	zassert_equal(smp_bt_client_attach(&peer, ATTACH_TIMEOUT), -ENOMEM,
		      "attach did not report the subscribe failure");
	zassert_false(gatt_stub_is_subscribed(), "a failed subscribe left a node linked");
	zassert_equal(gatt_stub_log.unsubscribe_calls, 0, "unsubscribed a node that never linked");
	zassert_equal(peer.ref, 0, "a failed attach leaked a connection reference");

	gatt_stub_faults.subscribe_err = 0;
	attach_ok();
}

/*
 * The peer answering the descriptor write with an ATT error is how a peer that demands an
 * encrypted or authenticated link presents itself.
 */
ZTEST(mcumgr_transport_bt_client_gatt, test_attach_fails_when_the_peer_rejects_the_subscription)
{
	gatt_stub_faults.ccc_att_err = BT_ATT_ERR_AUTHENTICATION;

	zassert_equal(smp_bt_client_attach(&peer, ATTACH_TIMEOUT), -EIO,
		      "attach ignored the ATT error on the descriptor write");
	zassert_false(smp_bt_client_is_attached(), "left attached after a rejected subscription");
	zassert_false(gatt_stub_is_subscribed(),
		      "left a node linked after a rejected subscription");
	zassert_equal(peer.ref, 0, "a failed attach leaked a connection reference");

	gatt_stub_faults.ccc_att_err = 0U;
	attach_ok();
}

/*
 * The host keeps writing to the discovery parameter block until it delivers a terminal
 * callback, so an attach that gives up on its timeout leaves that block live and the next
 * attach has to refuse rather than rewrite it.
 */
ZTEST(mcumgr_transport_bt_client_gatt, test_attach_timeout_refuses_the_next_one_until_settled)
{
	int rc;

	gatt_stub_faults.rsp_delay_ms = 300;

	rc = smp_bt_client_attach(&peer, K_MSEC(50));
	zassert_equal(rc, -ETIMEDOUT, "attach returned %d, expected -ETIMEDOUT", rc);
	zassert_false(smp_bt_client_is_attached(), "a timed out attach left the transport ready");

	zassert_equal(smp_bt_client_attach(&peer, K_MSEC(50)), -EBUSY,
		      "attach rewrote a discovery block the host still owns");

	/* Three legs, 300 ms apart, so the abandoned discovery ends about here. */
	k_sleep(K_MSEC(1200));
	zassert_false(smp_bt_client_is_attached(),
		      "a late discovery callback attached the transport on its own");

	gatt_stub_faults.rsp_delay_ms = 5;
	rc = smp_bt_client_attach(&peer, ATTACH_TIMEOUT);
	zassert_equal(rc, 0, "attach after a timed out attach failed (%d)", rc);
	zassert_equal(peer.ref, 1, "wrong connection reference count (%d)", peer.ref);
}

/*
 * The hardest case in the transport. An attach that gives up while the descriptor write
 * is still in flight leaves the host holding the subscription parameters as that write's
 * user data. The next attach must refuse rather than rewrite them, and when the write is
 * finally answered its verdict must not be reported to anyone.
 */
ZTEST(mcumgr_transport_bt_client_gatt, test_a_late_descriptor_response_is_not_the_next_verdict)
{
	int rc;

	gatt_stub_faults.rsp_delay_ms = 0;
	gatt_stub_faults.defer_ccc_rsp = true;

	rc = smp_bt_client_attach(&peer, K_MSEC(50));
	zassert_equal(rc, -ETIMEDOUT, "attach returned %d, expected -ETIMEDOUT", rc);
	zassert_true(gatt_stub_ccc_response_pending(), "the stub is not holding a write response");

	/* The failed attach unsubscribed, which issued a descriptor write of its own,
	 * still carrying the same parameters. Attach has to fail closed on that.
	 */
	zassert_equal(gatt_stub_log.unsubscribe_calls, 1, "the failed attach did not unsubscribe");
	zassert_equal(smp_bt_client_attach(&peer, K_MSEC(50)), -EBUSY,
		      "attach rewrote parameters the host still owns");

	/* The peer finally answers. Nothing is waiting for it. */
	gatt_stub_faults.defer_ccc_rsp = false;
	gatt_stub_deliver_ccc_response(BT_ATT_ERR_SUCCESS);
	zassert_false(smp_bt_client_is_attached(),
		      "a late descriptor write response attached the transport on its own");
	zassert_false(gatt_stub_is_subscribed(), "the node is still linked");

	gatt_stub_faults.rsp_delay_ms = 5;
	rc = smp_bt_client_attach(&peer, ATTACH_TIMEOUT);
	zassert_equal(rc, 0, "attach after a late write response failed (%d)", rc);
	zassert_equal(peer.ref, 1, "wrong connection reference count (%d)", peer.ref);
}

/*
 * An unsubscribe the host refuses leaves the node in its list, and rewriting a linked
 * node corrupts that list. Attach must refuse until the host lets go of it, which for a
 * link that never comes back means the disconnect sweep.
 */
ZTEST(mcumgr_transport_bt_client_gatt, test_a_failed_unsubscribe_blocks_attach_until_swept)
{
	attach_ok();

	gatt_stub_faults.unsubscribe_err = -ENOMEM;
	smp_bt_client_detach();
	zassert_equal(gatt_stub_log.unsubscribe_calls, 1, "detach did not unsubscribe");
	zassert_true(gatt_stub_is_subscribed(), "the stub unlinked a node it said it could not");

	zassert_equal(smp_bt_client_attach(&peer, ATTACH_TIMEOUT), -EBUSY,
		      "attach rewrote a node that is still in the host's list");

	/* The link goes away, and the host sweeps its volatile subscriptions. */
	gatt_stub_disconnect(&peer);
	zassert_false(gatt_stub_is_subscribed(), "the sweep did not unlink the node");

	peer.connected = true;
	gatt_stub_faults.unsubscribe_err = 0;
	attach_ok();
}

ZTEST(mcumgr_transport_bt_client_gatt, test_disconnect_detaches_and_releases_the_connection)
{
	attach_ok();

	gatt_stub_disconnect(&peer);

	zassert_false(smp_bt_client_is_attached(), "still ready after the peer disconnected");
	zassert_equal(peer.ref, 0, "the disconnect leaked a connection reference (ref %d)",
		      peer.ref);
	zassert_equal(transport()->functions.get_mtu(NULL), 0, "advertised an MTU with no target");
}

/* The link dropping while attach is parked on the descriptor write response. */
ZTEST(mcumgr_transport_bt_client_gatt, test_disconnect_while_attaching_is_reported_to_the_caller)
{
	int rc;

	gatt_stub_faults.rsp_delay_ms = 0;
	gatt_stub_faults.defer_ccc_rsp = true;

	k_work_schedule(&disconnect_work, K_MSEC(50));
	rc = smp_bt_client_attach(&peer, K_SECONDS(2));

	zassert_true(rc == -ENOTCONN || rc == -EIO,
		     "attach returned %d, expected -ENOTCONN or -EIO", rc);
	zassert_false(smp_bt_client_is_attached(), "attached to a peer that had gone away");
	zassert_equal(peer.ref, 0, "the failed attach leaked a connection reference (ref %d)",
		      peer.ref);
	zassert_false(gatt_stub_is_subscribed(), "left a node linked on a dead connection");

	/* And the transport is not wedged: a new link attaches. */
	peer.connected = true;
	gatt_stub_faults.defer_ccc_rsp = false;
	gatt_stub_faults.rsp_delay_ms = 5;
	attach_ok();
}

ZTEST(mcumgr_transport_bt_client_gatt, test_transmit_fragments_on_the_att_mtu)
{
	uint8_t pattern[300];
	int rc;

	gatt_stub_set_mtu(SMALL_MTU);
	attach_ok();

	for (size_t i = 0; i < sizeof(pattern); i++) {
		pattern[i] = (uint8_t)i;
	}

	rc = transmit(pattern, sizeof(pattern));
	zassert_equal(rc, MGMT_ERR_EOK, "transmit failed (%d)", rc);

	zassert_equal(gatt_stub_log.frag_count, 5, "expected five fragments, saw %d",
		      gatt_stub_log.frag_count);
	zassert_equal(gatt_stub_log.frag_len[0], SMALL_FRAG, "wrong first fragment size");
	zassert_equal(gatt_stub_log.frag_len[4], sizeof(pattern) - (4U * SMALL_FRAG),
		      "wrong last fragment size");
	zassert_equal(gatt_stub_log.last_write_handle, gatt_stub_peer.value_handle,
		      "wrote to the wrong handle");
	zassert_equal(gatt_stub_log.stream_len, sizeof(pattern), "wrong byte count on the wire");
	zassert_mem_equal(gatt_stub_log.stream, pattern, sizeof(pattern),
			  "the fragments do not reassemble to the original packet");
}

/*
 * A write the host refuses part way through leaves the peer holding an incomplete packet.
 * The transport has to stop, return the credit it took, and stay usable.
 */
ZTEST(mcumgr_transport_bt_client_gatt, test_transmit_stops_and_recovers_when_a_write_is_refused)
{
	uint8_t pattern[300];
	int rc;

	gatt_stub_set_mtu(SMALL_MTU);
	attach_ok();

	memset(pattern, 0x5a, sizeof(pattern));
	gatt_stub_faults.write_err = -ENOMEM;
	gatt_stub_faults.write_err_after = 2;

	rc = transmit(pattern, sizeof(pattern));
	zassert_equal(rc, MGMT_ERR_ENOMEM, "transmit returned %d, expected MGMT_ERR_ENOMEM", rc);
	zassert_equal(gatt_stub_log.frag_count, 2, "kept writing past the refusal (%d fragments)",
		      gatt_stub_log.frag_count);

	/* The credit the refused fragment took has to have come back. */
	gatt_stub_faults.write_err = 0;
	gatt_stub_log.frag_count = 0;
	rc = transmit(pattern, sizeof(pattern));
	zassert_equal(rc, MGMT_ERR_EOK, "the next transmit failed (%d)", rc);
	zassert_equal(gatt_stub_log.frag_count, 5, "wrong fragment count after recovery (%d)",
		      gatt_stub_log.frag_count);
}

/*
 * Nothing has been committed to the peer before the first fragment, so a packet that
 * cannot get a credit is dropped rather than held, and its buffer goes back to the pool.
 */
ZTEST(mcumgr_transport_bt_client_gatt, test_transmit_drops_an_unstarted_packet_with_no_credits)
{
	uint8_t prime[SMALL_FRAG * CONFIG_MCUMGR_TRANSPORT_BT_CLIENT_TX_CREDITS];
	uint8_t pattern[SMALL_FRAG];
	size_t before;
	int64_t start;
	int rc;

	gatt_stub_set_mtu(SMALL_MTU);
	attach_ok();

	gatt_stub_faults.defer_tx_done = true;
	memset(prime, 0x11, sizeof(prime));
	memset(pattern, 0x22, sizeof(pattern));

	/* Takes every credit and returns none. */
	zassert_equal(transmit(prime, sizeof(prime)), MGMT_ERR_EOK, "priming transmit failed");
	zassert_equal(gatt_stub_log.frag_count, CONFIG_MCUMGR_TRANSPORT_BT_CLIENT_TX_CREDITS,
		      "priming transmit wrote %d fragments", gatt_stub_log.frag_count);

	before = smp_packet_buffers_available();
	start = k_uptime_get();
	rc = transmit(pattern, sizeof(pattern));

	zassert_not_equal(rc, MGMT_ERR_EOK, "transmit claimed success with no credits");
	zassert_true(k_uptime_get() - start >= CONFIG_MCUMGR_TRANSPORT_BT_CLIENT_TX_CREDIT_TIMEOUT,
		     "gave up before the configured credit timeout");
	zassert_equal(gatt_stub_log.frag_count, CONFIG_MCUMGR_TRANSPORT_BT_CLIENT_TX_CREDITS,
		      "wrote a fragment it had no credit for");
	zassert_equal(smp_packet_buffers_available(), before,
		      "the starved transmit did not release the packet");

	gatt_stub_flush_tx_done();
}

/*
 * Credits that the old link's completion callbacks never returned died with that link,
 * and the host does not run them once the bearer is gone.
 */
ZTEST(mcumgr_transport_bt_client_gatt, test_attach_replenishes_credits_lost_with_the_old_link)
{
	uint8_t prime[SMALL_FRAG * CONFIG_MCUMGR_TRANSPORT_BT_CLIENT_TX_CREDITS];

	gatt_stub_set_mtu(SMALL_MTU);
	attach_ok();

	gatt_stub_faults.defer_tx_done = true;
	memset(prime, 0x33, sizeof(prime));
	zassert_equal(transmit(prime, sizeof(prime)), MGMT_ERR_EOK, "priming transmit failed");

	gatt_stub_disconnect(&peer);
	zassert_false(smp_bt_client_is_attached(), "still attached after the disconnect");

	peer.connected = true;
	gatt_stub_faults.defer_tx_done = false;
	gatt_stub_log.frag_count = 0;
	attach_ok();

	zassert_equal(transmit(prime, sizeof(prime)), MGMT_ERR_EOK,
		      "the transmit after re-attaching was starved of credits");
	zassert_equal(gatt_stub_log.frag_count, CONFIG_MCUMGR_TRANSPORT_BT_CLIENT_TX_CREDITS,
		      "wrong fragment count after re-attaching (%d)", gatt_stub_log.frag_count);
}

/*
 * The whole path, end to end through the real code: the SMP client sends a command on
 * the MCUmgr work queue, the transport fragments it, the peer's answer arrives as
 * notifications, the real reassembly context puts it back together and the real SMP core
 * routes it to the client that asked for it.
 */
ZTEST(mcumgr_transport_bt_client_gatt, test_a_real_smp_response_reaches_the_client_that_asked)
{
	uint8_t body[40];
	uint8_t response[64];
	uint16_t response_len;

	gatt_stub_set_mtu(SMALL_MTU);
	attach_ok();

	send_command();

	/* The command really did go out, in fragments, to the discovered value handle. */
	zassert_equal(gatt_stub_log.frag_count, 2, "expected two request fragments, saw %d",
		      gatt_stub_log.frag_count);
	zassert_equal(gatt_stub_log.frag_len[0], SMALL_FRAG, "wrong first fragment size");
	zassert_equal(gatt_stub_log.last_write_handle, gatt_stub_peer.value_handle,
		      "the request went to the wrong handle");
	zassert_equal(gatt_stub_log.stream_len, sizeof(struct smp_hdr) + 100U,
		      "wrong request length on the wire (%u)", gatt_stub_log.stream_len);

	fill_body(body, sizeof(body), 'r');
	response_len = build_response(response, sizeof(response), body, sizeof(body), 0);
	notify_fragmented(response, response_len, RSP_FRAG);
	k_sleep(K_MSEC(SETTLE_MS));

	zassert_equal(rsp_count, 1, "the client got %d responses, expected one", rsp_count);
	zassert_equal(rsp_timeouts, 0, "the command timed out as well");
	zassert_equal_ptr(rsp_user_data, &client_user_data, "the user data did not come back");
	zassert_equal(rsp_payload_len, sizeof(body), "wrong response payload length (%u)",
		      rsp_payload_len);
	zassert_mem_equal(rsp_payload, body, sizeof(body), "wrong response payload");
}

ZTEST(mcumgr_transport_bt_client_gatt, test_an_existing_subscription_needs_no_descriptor_write)
{
	int rc;

	/* Another subscription on the connection already has notifications enabled for
	 * the handle, so the host links the node without a descriptor write and
	 * delivers no response for it. Attach must not wait for one.
	 */
	gatt_stub_faults.subscribe_already_enabled = true;

	rc = smp_bt_client_attach(&peer, K_MSEC(200));
	zassert_equal(rc, 0, "attach waited for a write the host never sent (rc %d)", rc);
	zassert_true(smp_bt_client_is_attached(), "not attached");

	smp_bt_client_detach();
	zassert_false(smp_bt_client_is_attached(), "still attached after detach");

	/* The unsubscribe sent no write either, so nothing is left for the next attach
	 * to be refused over.
	 */
	rc = smp_bt_client_attach(&peer, K_MSEC(200));
	zassert_equal(rc, 0, "re-attach refused after an unsubscribe without a write (rc %d)", rc);
	smp_bt_client_detach();
}

ZTEST(mcumgr_transport_bt_client_gatt, test_a_response_for_another_sequence_number_is_dropped)
{
	uint8_t body[40];
	uint8_t response[64];
	uint16_t response_len;

	gatt_stub_set_mtu(SMALL_MTU);
	attach_ok();

	send_command();

	fill_body(body, sizeof(body), 's');
	response_len = build_response(response, sizeof(response), body, sizeof(body), 1);
	notify_fragmented(response, response_len, RSP_FRAG);
	k_sleep(K_MSEC(SETTLE_MS));

	zassert_equal(rsp_count, 0,
		      "a response with the wrong sequence number completed a command");

	/* The right one, right after it, still gets through. */
	response_len = build_response(response, sizeof(response), body, sizeof(body), 0);
	notify_fragmented(response, response_len, RSP_FRAG);
	k_sleep(K_MSEC(SETTLE_MS));

	zassert_equal(rsp_count, 1, "the path did not recover from an unmatched response");
}

/*
 * A first fragment too short to hold an SMP header cannot be framed at all. The peer that
 * sends one has an ATT_MTU below what SMP needs, and the transport has to drop it without
 * leaving anything behind.
 */
ZTEST(mcumgr_transport_bt_client_gatt, test_a_first_fragment_too_short_to_frame_is_dropped)
{
	uint8_t body[40];
	uint8_t response[64];
	uint16_t response_len;

	gatt_stub_set_mtu(SMALL_MTU);
	attach_ok();
	send_command();

	fill_body(body, sizeof(body), 't');
	response_len = build_response(response, sizeof(response), body, sizeof(body), 0);
	notify_fragmented(response, response_len, sizeof(struct smp_hdr) - 1U);
	k_sleep(K_MSEC(SETTLE_MS));

	zassert_equal(rsp_count, 0, "an unframeable response was delivered to the client");

	/* And the same answer, sent again at a workable fragment size, arrives whole. */
	notify_fragmented(response, response_len, RSP_FRAG);
	k_sleep(K_MSEC(SETTLE_MS));

	zassert_equal(rsp_count, 1, "the path did not recover from an unframeable response");
	zassert_mem_equal(rsp_payload, body, sizeof(body), "the response body was misframed");
}

/*
 * A first fragment whose header claims more than the reassembly buffer can ever hold is
 * refused before anything is allocated, and must not leave the context half built.
 */
ZTEST(mcumgr_transport_bt_client_gatt, test_an_impossible_response_length_does_not_wedge_the_path)
{
	uint8_t body[40];
	uint8_t response[64];
	uint16_t response_len;
	struct smp_hdr hdr;

	gatt_stub_set_mtu(SMALL_MTU);
	attach_ok();
	send_command();

	fill_body(body, sizeof(body), 'u');
	response_len = build_response(response, sizeof(response), body, sizeof(body), 0);
	memcpy(&hdr, response, sizeof(hdr));
	hdr.nh_len = sys_cpu_to_be16(CONFIG_MCUMGR_TRANSPORT_NETBUF_SIZE);
	memcpy(response, &hdr, sizeof(hdr));

	notify_fragmented(response, response_len, RSP_FRAG);
	k_sleep(K_MSEC(SETTLE_MS));
	zassert_equal(rsp_count, 0, "an impossible response was delivered to the client");

	/* And the genuine answer, sent right afterwards, still arrives intact. */
	response_len = build_response(response, sizeof(response), body, sizeof(body), 0);
	notify_fragmented(response, response_len, RSP_FRAG);
	k_sleep(K_MSEC(SETTLE_MS));

	zassert_equal(rsp_count, 1, "the path did not recover from an impossible response");
	zassert_mem_equal(rsp_payload, body, sizeof(body), "the response body was misframed");
}

/*
 * A packet the peer never finished must not be left in the reassembly context for the
 * next link to append to, which would misframe its first real response.
 */
ZTEST(mcumgr_transport_bt_client_gatt, test_a_half_delivered_response_does_not_misframe_the_next)
{
	uint8_t body[40];
	uint8_t response[64];
	uint16_t response_len;

	gatt_stub_set_mtu(SMALL_MTU);
	attach_ok();
	send_command();

	/* Half a response, then the link drops with the rest of it still owed. */
	fill_body(body, sizeof(body), 'v');
	response_len = build_response(response, sizeof(response), body, sizeof(body), 0);
	gatt_stub_notify(response, RSP_FRAG);
	gatt_stub_disconnect(&peer);
	k_sleep(K_MSEC(SETTLE_MS));

	/* The same command is still outstanding, so the peer's answer to it, delivered
	 * whole over a new link, has to arrive as itself and not as the tail of the
	 * fragment that was abandoned.
	 */
	peer.connected = true;
	attach_ok();
	notify_fragmented(response, response_len, RSP_FRAG);
	k_sleep(K_MSEC(SETTLE_MS));

	zassert_equal(rsp_count, 1, "the response after a half delivered one did not arrive");
	zassert_equal(rsp_payload_len, sizeof(body), "the response was misframed (%u bytes)",
		      rsp_payload_len);
	zassert_mem_equal(rsp_payload, body, sizeof(body), "the response body was misframed");
}

/*
 * The transport leaves the ready state before it unsubscribes, and the peer keeps
 * notifying until its descriptor is actually written. A fragment that arrives in that
 * window must be discarded rather than collected, or it becomes the head of a partial
 * packet that nothing ever clears.
 */
ZTEST(mcumgr_transport_bt_client_gatt, test_a_notification_while_detached_is_discarded)
{
	uint8_t body[40];
	uint8_t response[64];
	uint16_t response_len;

	gatt_stub_set_mtu(SMALL_MTU);
	attach_ok();
	send_command();

	fill_body(body, sizeof(body), 'w');
	response_len = build_response(response, sizeof(response), body, sizeof(body), 0);

	/* The host cannot unlink the node, so notifications keep being routed to the
	 * transport after it has stopped being a target.
	 */
	gatt_stub_faults.unsubscribe_err = -ENOMEM;
	smp_bt_client_detach();
	zassert_true(gatt_stub_is_subscribed(), "the node was unlinked after all");

	notify_fragmented(response, response_len, RSP_FRAG);
	k_sleep(K_MSEC(SETTLE_MS));
	zassert_equal(rsp_count, 0, "a notification arriving while detached was processed");

	/* Nothing was left half collected: the same response, delivered again over a new
	 * link, arrives whole.
	 */
	gatt_stub_disconnect(&peer);
	peer.connected = true;
	gatt_stub_faults.unsubscribe_err = 0;
	attach_ok();
	notify_fragmented(response, response_len, RSP_FRAG);
	k_sleep(K_MSEC(SETTLE_MS));

	zassert_equal(rsp_count, 1, "the reassembly context was left holding a stale fragment");
	zassert_mem_equal(rsp_payload, body, sizeof(body), "the response body was misframed");
}

static void *suite_setup(void)
{
	int rc;

	gatt_stub_reset();
	k_work_init_delayable(&disconnect_work, disconnect_work_handler);

	rc = smp_client_object_init(&bt_client, SMP_BLUETOOTH_CLIENT_TRANSPORT);
	zassert_equal(rc, MGMT_ERR_EOK, "the Bluetooth client transport did not register (%d)", rc);
	zassert_equal_ptr(bt_client.smpt, transport(), "the client bound to the wrong transport");

	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	gatt_stub_reset();
	memset(&peer, 0, sizeof(peer));
	peer.connected = true;

	memset(rsp_payload, 0, sizeof(rsp_payload));
	rsp_payload_len = 0;
	rsp_count = 0;
	rsp_timeouts = 0;
	cmds_sent = 0;
	rsp_user_data = NULL;
}

/*
 * Puts the transport, the host and the SMP client back to a known state. A disconnect is
 * the only thing that unconditionally ends every claim the host can have on the
 * transport, so the teardown ends with one whether or not the test used the link.
 */
static void after(void *fixture)
{
	struct k_work_sync sync;

	ARG_UNUSED(fixture);

	(void)k_work_cancel_delayable_sync(&disconnect_work, &sync);

	gatt_stub_faults.write_err = 0;
	gatt_stub_faults.unsubscribe_err = 0;
	gatt_stub_faults.defer_ccc_rsp = false;
	gatt_stub_faults.defer_tx_done = false;

	smp_bt_client_detach();
	gatt_stub_flush_tx_done();
	k_sleep(K_MSEC(SETTLE_MS));

	peer.connected = true;
	gatt_stub_disconnect(&peer);
	k_sleep(K_MSEC(SETTLE_MS));

	/* A command a test left outstanding still holds a buffer from the shared pool.
	 * Wait for the SMP client to give up on it rather than starve the next test.
	 */
	for (int i = 0; i < 40 && cmds_sent > (rsp_count + rsp_timeouts); i++) {
		k_sleep(K_MSEC(50));
	}
}

ZTEST_SUITE(mcumgr_transport_bt_client_gatt, NULL, suite_setup, before, after, NULL);
