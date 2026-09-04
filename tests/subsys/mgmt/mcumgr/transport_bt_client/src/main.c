/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Jeff Welder
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Coverage for the Bluetooth central MCUmgr SMP client transport that does not need a
 * radio. The Bluetooth host is built but never enabled and no controller is present, so
 * what is exercised here is everything the transport does before a link exists, plus
 * everything it has to keep doing safely once one is gone.
 *
 * Service discovery, the subscription, splitting a packet onto ATT writes and turning
 * notifications back into SMP responses all need a peer running the SMP server, so they
 * are not covered here and belong in a simulated radio test.
 */

#include <errno.h>
#include <stddef.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include <zephyr/mgmt/mcumgr/mgmt/mgmt_defines.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zephyr/mgmt/mcumgr/smp/smp_client.h>
#include <zephyr/mgmt/mcumgr/transport/smp.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt_client.h>

#if !defined(CONFIG_MCUMGR_TRANSPORT_BT_CLIENT)
#error "Expected Kconfig option CONFIG_MCUMGR_TRANSPORT_BT_CLIENT not enabled"
#endif

/*
 * The transport drives the shared packet reassembly context, and smp_transport_init()
 * only sets that context up when this option is on. The option has no prompt, so it
 * cannot be turned on from a project configuration: the transport's own Kconfig has to
 * select it. A build that inherited it from the SMP server transport instead would run
 * with an uninitialised context as soon as the server was turned off.
 */
#if !defined(CONFIG_MCUMGR_TRANSPORT_REASSEMBLY)
#error "CONFIG_MCUMGR_TRANSPORT_BT_CLIENT must select CONFIG_MCUMGR_TRANSPORT_REASSEMBLY"
#endif

/*
 * The transport type is public ABI. Types up to 63 are reserved for in-tree transports
 * and 64 is the slot out-of-tree users are told to take, so neither that one nor the type
 * the SMP server transport claims is available here. The server's type in particular
 * cannot be shared: the client registry is first wins, so the loser of that race does not
 * register at all and every user of it fails to bind.
 */
BUILD_ASSERT(SMP_BLUETOOTH_CLIENT_TRANSPORT != SMP_BLUETOOTH_TRANSPORT,
	     "Bluetooth client transport must not reuse the SMP server transport type");
BUILD_ASSERT(SMP_BLUETOOTH_CLIENT_TRANSPORT < SMP_USER_DEFINED_TRANSPORT,
	     "Bluetooth client transport must not take the user defined transport type");

static struct smp_transport *client_transport(void)
{
	return smp_client_transport_get(SMP_BLUETOOTH_CLIENT_TRANSPORT);
}

/*
 * The transport registers itself from its MCUmgr handler at boot, so by the time any test
 * runs the registry has to hand it back with a complete function table. A transport that
 * failed to register is not merely absent: smp_client_object_init() would fail for every
 * user of it.
 */
ZTEST(mcumgr_transport_bt_client, test_transport_registered_at_boot)
{
	struct smp_transport *smpt = client_transport();

	zassert_not_null(smpt, "Bluetooth client transport did not register itself");
	zassert_not_null(smpt->functions.output, "Transport registered with no transmit function");
	zassert_not_null(smpt->functions.get_mtu, "Transport registered with no MTU function");
}

#if defined(CONFIG_MCUMGR_TRANSPORT_BT)
/*
 * A device may serve SMP to one peer while driving another, which only works while the
 * two transports are separate objects under separate types. The SMP server registers
 * itself as a client transport as well whenever the SMP client is built, so this is the
 * configuration where a shared type would have gone unnoticed.
 */
ZTEST(mcumgr_transport_bt_client, test_server_and_client_transports_are_distinct)
{
	struct smp_transport *server = smp_client_transport_get(SMP_BLUETOOTH_TRANSPORT);
	struct smp_transport *client = client_transport();

	zassert_not_null(server, "SMP server transport did not register itself");
	zassert_not_null(client, "Bluetooth client transport did not register itself");
	zassert_true(server != client,
		     "SMP server and Bluetooth client resolved to the same transport");
}
#endif

/* An SMP client asks for the transport by type, which is the only way it ever reaches it. */
ZTEST(mcumgr_transport_bt_client, test_smp_client_binds_to_transport)
{
	struct smp_client_object client;
	int rc;

	memset(&client, 0, sizeof(client));

	rc = smp_client_object_init(&client, SMP_BLUETOOTH_CLIENT_TRANSPORT);
	zassert_equal(rc, MGMT_ERR_EOK, "SMP client could not bind to the transport (err %d)", rc);
	zassert_equal(client.smpt, client_transport(),
		      "SMP client bound to a different transport than the one registered");
}

/*
 * A request buffer allocated through a bound client has to come back to the pool, which
 * covers the transport having no user data hooks: the core copies user data by value for
 * it, and the client's own free path never calls a hook.
 */
ZTEST(mcumgr_transport_bt_client, test_smp_client_request_buffer_round_trip)
{
	struct smp_client_object client;
	struct net_buf *nb;
	size_t available;

	memset(&client, 0, sizeof(client));
	zassert_equal(smp_client_object_init(&client, SMP_BLUETOOTH_CLIENT_TRANSPORT), MGMT_ERR_EOK,
		      "SMP client could not bind to the transport");

	available = smp_packet_buffers_available();
	zassert_true(available > 0, "No packet buffers available before the test ran");

	nb = smp_client_buf_allocation(&client, MGMT_GROUP_ID_OS, 0, MGMT_OP_READ,
				       SMP_MCUMGR_VERSION_1);
	zassert_not_null(nb, "SMP client could not allocate a request buffer");
	zassert_equal(smp_packet_buffers_available(), available - 1,
		      "Allocating a request did not take a buffer from the pool");

	smp_client_buf_free(nb);
	zassert_equal(smp_packet_buffers_available(), available,
		      "Freeing a request did not return its buffer to the pool");
}

ZTEST(mcumgr_transport_bt_client, test_not_attached_at_boot)
{
	zassert_false(smp_bt_client_is_attached(), "Transport reported a target before any attach");
}

ZTEST(mcumgr_transport_bt_client, test_attach_rejects_null_connection)
{
	int rc = smp_bt_client_attach(NULL, K_NO_WAIT);

	zassert_equal(rc, -EINVAL, "Attaching without a connection returned %d, expected -EINVAL",
		      rc);
	zassert_false(smp_bt_client_is_attached(), "A rejected attach left a target behind");
}

/*
 * Detach is documented as always safe to call, including when nothing is attached, so
 * that a caller can use it as an unconditional teardown step. It has real work to do even
 * then: it drops any partial reassembly and drains the receive queue.
 */
ZTEST(mcumgr_transport_bt_client, test_detach_when_idle_is_safe)
{
	smp_bt_client_detach();
	zassert_false(smp_bt_client_is_attached(), "Detaching an idle transport attached one");

	smp_bt_client_detach();
	zassert_false(smp_bt_client_is_attached(), "A second detach attached a target");

	/* Detach must not leave the transport wedged for the next caller. */
	zassert_equal(smp_bt_client_attach(NULL, K_NO_WAIT), -EINVAL,
		      "Attach stopped validating its arguments after a detach");
}

/*
 * With no target there is no connection to read an ATT MTU from, and 0 is how a transport
 * says it cannot transmit right now.
 */
ZTEST(mcumgr_transport_bt_client, test_mtu_is_zero_while_detached)
{
	struct smp_transport *smpt = client_transport();

	zassert_not_null(smpt, "Bluetooth client transport did not register itself");
	zassert_equal(smpt->functions.get_mtu(NULL), 0,
		      "Transport advertised an MTU with no target attached");
}

/*
 * The transmit function owns the packet on every return path, error paths included,
 * because the SMP core drops its own pointer to it as soon as the call returns. Refusing
 * to transmit is one of those paths, and it is the one a detach or a disconnect can push
 * a packet down at any time, since clearing the ready state does not withdraw a work item
 * that the MCUmgr work queue has already picked up.
 *
 * Running it far more times than the pool has buffers turns a single missed release into
 * a failure rather than a slow leak.
 */
ZTEST(mcumgr_transport_bt_client, test_transmit_while_detached_releases_the_packet)
{
	struct smp_transport *smpt = client_transport();
	size_t available;
	unsigned int i;

	zassert_not_null(smpt, "Bluetooth client transport did not register itself");
	zassert_false(smp_bt_client_is_attached(), "A target was attached before the test ran");

	available = smp_packet_buffers_available();
	zassert_true(available > 0, "No packet buffers available before the test ran");

	for (i = 0; i < (available * 4U); i++) {
		struct net_buf *nb = smp_packet_alloc();
		int rc;

		zassert_not_null(nb,
				 "Packet pool exhausted after %u transmits, the transmit "
				 "path leaked a buffer while refusing to send",
				 i);

		rc = smpt->functions.output(nb);
		zassert_not_equal(rc, MGMT_ERR_EOK,
				  "Transmit reported success with no target attached");
	}

	zassert_equal(smp_packet_buffers_available(), available,
		      "Refused transmits did not return every buffer to the pool");
}

ZTEST_SUITE(mcumgr_transport_bt_client, NULL, NULL, NULL, NULL, NULL);
