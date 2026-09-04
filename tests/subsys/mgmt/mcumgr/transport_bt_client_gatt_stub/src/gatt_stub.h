/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Jeff Welder
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_TESTS_SUBSYS_MGMT_MCUMGR_TRANSPORT_BT_CLIENT_GATT_STUB_H_
#define ZEPHYR_TESTS_SUBSYS_MGMT_MCUMGR_TRANSPORT_BT_CLIENT_GATT_STUB_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

/*
 * A scripted stand-in for the Bluetooth host's GATT client API, modelled on the state
 * machine in subsys/bluetooth/host/gatt.c rather than on what the unit under test happens
 * to want. See gatt_stub.c for which host behaviour each entry point reproduces.
 *
 * The real host is not compiled into this test, so these are the only definitions of the
 * GATT client entry points in the image, and no radio, controller or bt_enable() is
 * involved. What the stub can and cannot stand in for is spelled out in the test's
 * README-style comment at the top of main.c.
 */

#define GATT_STUB_MAX_FRAGS  64
#define GATT_STUB_STREAM_LEN 2048

/*
 * The connection object. The real one is opaque and owned by the host, so a test builds
 * its own and hands the address to the transport exactly as an application would hand it
 * one that came out of bt_conn_le_create().
 */
struct bt_conn {
	/* Reference count, so that a test can prove the transport balances its
	 * bt_conn_ref() and bt_conn_unref() calls on every path.
	 */
	int ref;
	bool connected;
};

/* The attribute layout the scripted peer answers discovery with. */
struct gatt_stub_peer {
	uint16_t svc_handle;
	uint16_t svc_end_handle;
	uint16_t chrc_handle;
	uint16_t value_handle;
	uint16_t ccc_handle;
	uint8_t properties;
	bool has_svc;
	bool has_chrc;
	bool has_ccc;
};

/* What the transport did, as seen from the host's side of the API. */
struct gatt_stub_log {
	int discover_calls;
	int subscribe_calls;
	int unsubscribe_calls;
	int write_calls;
	uint16_t last_write_handle;
	uint16_t frag_len[GATT_STUB_MAX_FRAGS];
	int frag_count;
	uint8_t stream[GATT_STUB_STREAM_LEN];
	uint16_t stream_len;
};

/* Everything a test can make the host do wrong. */
struct gatt_stub_faults {
	/* Returned by bt_gatt_discover() once discover_err_after calls have been
	 * accepted. The host delivers no callback at all for a request it never queued.
	 */
	int discover_err;
	int discover_err_after;
	/* Returned by bt_gatt_subscribe(); the node is not linked and no write goes out. */
	int subscribe_err;
	/* Returned by bt_gatt_unsubscribe(); the node stays linked, as in the host. */
	int unsubscribe_err;
	/* Returned by bt_gatt_write_without_response_cb() once write_err_after writes
	 * have been accepted.
	 */
	int write_err;
	int write_err_after;
	/* ATT error the client characteristic configuration write response carries. */
	uint8_t ccc_att_err;
	/* Another subscription on the connection already has notifications enabled for
	 * the handle: the host links or unlinks the node without a descriptor write, and
	 * completes an unsubscribe synchronously through notify(NULL).
	 */
	bool subscribe_already_enabled;
	/* Withhold that response until gatt_stub_deliver_ccc_response() is called. */
	bool defer_ccc_rsp;
	/* Withhold write completion callbacks until gatt_stub_flush_tx_done(). */
	bool defer_tx_done;
	/* How long the host takes to answer one leg of a discovery. Zero delivers the
	 * result from inside bt_gatt_discover(), which the real host never does.
	 */
	int rsp_delay_ms;
};

extern struct gatt_stub_peer gatt_stub_peer;
extern struct gatt_stub_log gatt_stub_log;
extern struct gatt_stub_faults gatt_stub_faults;

/* Returns the stub to a healthy peer with no faults injected. */
void gatt_stub_reset(void);

/* The ATT_MTU the stub reports for every connection. */
void gatt_stub_set_mtu(uint16_t mtu);

/* Runs the deferred client characteristic configuration write response by hand. */
void gatt_stub_deliver_ccc_response(uint8_t att_err);

/* True while a descriptor write response is still owed to the transport. */
bool gatt_stub_ccc_response_pending(void);

/* Runs withheld write completion callbacks. */
void gatt_stub_flush_tx_done(void);

/* Delivers a notification on the subscribed characteristic. */
void gatt_stub_notify(const void *data, uint16_t len);

/* The whole disconnect sequence, in the order the host runs it. */
void gatt_stub_disconnect(struct bt_conn *conn);

/* True while the transport's node is in the host's subscription list. */
bool gatt_stub_is_subscribed(void);

/* The subscription parameters the transport handed to bt_gatt_subscribe(). */
const struct bt_gatt_subscribe_params *gatt_stub_sub_params(void);

#endif /* ZEPHYR_TESTS_SUBSYS_MGMT_MCUMGR_TRANSPORT_BT_CLIENT_GATT_STUB_H_ */
