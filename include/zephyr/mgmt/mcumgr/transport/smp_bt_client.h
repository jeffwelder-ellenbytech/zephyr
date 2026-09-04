/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Jeff Welder
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Bluetooth GATT client transport for the MCUmgr SMP client.
 * @ingroup mcumgr_transport_bt_client
 */

#ifndef ZEPHYR_INCLUDE_MGMT_MCUMGR_TRANSPORT_SMP_BT_CLIENT_H_
#define ZEPHYR_INCLUDE_MGMT_MCUMGR_TRANSPORT_SMP_BT_CLIENT_H_

#include <stdbool.h>

#include <zephyr/kernel.h>

struct bt_conn;

/**
 * @brief Drive a Bluetooth peer that runs the SMP service from an MCUmgr SMP client,
 *        with the local device as the GATT client.
 * @defgroup mcumgr_transport_bt_client Bluetooth client transport
 * @ingroup mcumgr_transport
 * @since 4.5
 * @version 0.1.0
 * @{
 *
 * The transport is the counterpart of @ref mcumgr_transport_bt, which is the GATT server
 * side; this one is the GATT client side. It discovers the SMP service on a connected
 * peer, subscribes to its notifications, splits every outbound SMP packet into ATT_MTU-3
 * sized Write Without Response writes, and reassembles the notifications that come back
 * into complete SMP responses.
 *
 * The transport registers itself at boot under #SMP_BLUETOOTH_CLIENT_TRANSPORT, which is
 * a different transport type from the one the SMP server claims, so a device may be an
 * SMP server towards one peer and an SMP client towards another at the same time.
 *
 * Notifications are handed to the SMP core exactly as every other transport hands on what
 * it receives, so a request the peer sends is executed by whatever management groups the
 * image serves, and a peer that is itself an SMP client over smp_bt.c can drive this
 * device. Restrict the groups, or use the MCUmgr management hooks, if that is not wanted.
 *
 * A single target at a time is supported, and the restriction is structural rather than
 * advisory: there is one transport object, whose packet reassembly context and outgoing
 * connection are both singletons. smp_bt_client_attach() therefore refuses while another
 * target is attached, and the in-tree MCUmgr client groups already serialise their
 * commands on a per-group mutex.
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Discover the SMP service on a connected peer and subscribe to it, making
 *		the transport ready to carry SMP client traffic to that peer.
 *
 * The whole discover and subscribe sequence is carried out before this function returns,
 * so it blocks the calling thread and must not be called from a work queue item, and in
 * particular never from the MCUmgr work queue, which is the thread that later runs the
 * transport's transmit path.
 *
 * On success the transport holds a reference on @p conn until it is released by
 * smp_bt_client_detach() or by the peer disconnecting.
 *
 * @param conn		Connection to the peer that runs the SMP service.
 * @param timeout	How long to wait for the peer to answer. This bounds the waits for
 *			the discovery and subscription responses, and so the great
 *			majority of the call, but not the whole of it: issuing a GATT
 *			procedure can itself block on the Bluetooth host's shared ATT
 *			transmit buffers, which every GATT client entry point does and
 *			which takes no timeout of its own.
 *
 * @retval 0		The transport is attached and ready to carry SMP traffic.
 * @retval -EINVAL	@p conn is NULL.
 * @retval -ENODEV	The transport failed to register at boot and is unusable.
 * @retval -EBUSY	Another target is attached, or a previous attach has not settled
 *			because the host still owns its discovery, its subscription or its
 *			descriptor write. Retry later. The case that only clears on
 *			disconnect, an unsubscribe that failed outright or a descriptor
 *			write answered with an error after the subscription is gone, is
 *			bounded by dropping the link.
 * @retval -ENOTSUP	The peer does not expose an SMP service, or the service does not
 *			carry a notifiable SMP characteristic that can be written without
 *			a response.
 * @retval -ENOTCONN	The peer disconnected while attaching.
 * @retval -ETIMEDOUT	Discovery or the client characteristic configuration write did not
 *			complete within @p timeout.
 * @retval -EIO		The peer rejected the client characteristic configuration write,
 *			usually because its SMP characteristic requires an encrypted or
 *			authenticated link.
 *
 * @return		Other negative errno reported by the Bluetooth host.
 */
int smp_bt_client_attach(struct bt_conn *conn, k_timeout_t timeout);

/**
 * @brief	Release the current target.
 *
 * Best effort and always safe to call, including when nothing is attached: the transport
 * stops accepting traffic, unsubscribes if the link is still up, discards any partially
 * reassembled response together with any response still queued for processing, and
 * releases the connection reference taken by smp_bt_client_attach().
 *
 * Like smp_bt_client_attach() this may block, and must not be called from the MCUmgr work
 * queue.
 */
void smp_bt_client_detach(void);

/**
 * @brief	Report whether a target is attached.
 *
 * @retval true		A target is attached and the transport can carry SMP traffic.
 * @retval false	No target is attached.
 */
bool smp_bt_client_is_attached(void);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_MGMT_MCUMGR_TRANSPORT_SMP_BT_CLIENT_H_ */
