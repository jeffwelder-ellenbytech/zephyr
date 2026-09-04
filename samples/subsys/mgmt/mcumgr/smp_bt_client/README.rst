.. zephyr:code-sample:: smp-bt-client
   :name: SMP client (Bluetooth)
   :relevant-api: mcumgr_transport_bt_client

   Upload a firmware image from a file system to a Bluetooth peer that runs the SMP service.

Overview
********

This sample scans for a Bluetooth peer that advertises the MCUmgr SMP service, connects to
it, attaches the MCUmgr SMP client Bluetooth transport with :c:func:`smp_bt_client_attach`,
checks the link with an ``os_mgmt`` echo command, and then uploads a firmware image into the
peer's MCUboot secondary slot with the ``img_mgmt`` upload client.

The image is read from a littlefs file system on the board's external flash rather than from
an array compiled into the binary, which is the part worth copying: the update pushed to the
peer is data on a file system, so it can be replaced without rebuilding this application. The
board is the source of the update, not its target, so it does not run MCUboot itself.

Requirements
************

* An :zephyr:board:`nrf52840dk` or an :zephyr:board:`nrf54lm20dk`. The board overlay in
  :file:`boards/` carves a 1 MiB littlefs partition mounted at ``/lfs1`` out of the on-board
  MX25R64 that both carry, over QSPI on the first and SPI on the second. Another board needs
  an overlay of its own providing an equivalent ``lfs1``
  :dtcompatible:`zephyr,fstab,littlefs` entry.

* A second board running the :zephyr:code-sample:`smp-svr` sample, built with sysbuild so
  that MCUboot is present and ``img_mgmt`` has a secondary slot to upload into, and with
  :file:`bt.conf` so that it advertises the SMP service and accepts SMP over Bluetooth
  without pairing.

The peer's :kconfig:option:`CONFIG_MCUMGR_TRANSPORT_NETBUF_SIZE` must be at least as large as
this sample's, which is 2048, because the upload client sizes every frame from its own value
without consulting the transport MTU. The ``smp_svr`` :file:`bt.conf` uses 2475.

Building and Running
********************

Build and flash the peer first:

.. zephyr-app-commands::
   :tool: west
   :zephyr-app: samples/subsys/mgmt/mcumgr/smp_svr
   :board: nrf52840dk/nrf52840
   :goals: build flash
   :west-args: --sysbuild
   :gen-args: -DEXTRA_CONF_FILE="bt.conf"
   :compact:

MCUboot is not optional here: the secondary slot ``img_mgmt`` uploads into comes from the
bootloader's partition layout, so a peer built without sysbuild rejects the first frame.

Then build and flash this sample on a second board:

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/mgmt/mcumgr/smp_bt_client
   :board: nrf52840dk/nrf52840
   :goals: build flash
   :compact:

Running the pair
================

Open a console on each board at 115200 baud, and bring the peer up first. This sample scans
for 30 seconds and then gives up without trying again, so reset the client only once the peer
has logged ``Advertising successfully started``. The scan is passive, so the peer is
recognised by the SMP service UUID in its advertising data and by nothing else, and
duplicates are filtered: a peer whose first connection attempt failed is not reported again.

Getting an image onto the file system
=====================================

The sample uploads whatever it finds at :file:`/lfs1/update.bin`. On first boot the external
flash is unformatted, so littlefs formats it and the file does not exist; rather than stop,
the sample writes a 4 KiB placeholder of its own, a valid MCUboot image header followed by
zeros, so that the whole flow runs out of the box. It is not bootable, so do not mark it
pending on the peer.

To upload a real one, put :file:`build/smp_svr/zephyr/zephyr.signed.bin` from the peer's
build on the file system as :file:`/lfs1/update.bin` and reset the board. These options make
the board an SMP server towards a host while it stays an SMP client towards the peer, so the
file can be written over the serial port as the :zephyr:code-sample:`smp-svr` README's file
system section describes:

.. code-block:: cfg

   CONFIG_BASE64=y
   CONFIG_SHELL=y
   CONFIG_SHELL_BACKEND_SERIAL=y
   CONFIG_MCUMGR_TRANSPORT_SHELL=y
   CONFIG_MCUMGR_GRP_FS=y
   CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE=2304

Do it with the peer powered off, so that the scan expires and leaves the console to the
shell. Such a build hands any host on the serial port unrestricted access to the file system,
so keep it to provisioning and use the MCUmgr file access hooks described in
:ref:`mcumgr_callbacks` for anything more. Programming the application does not touch the
external flash, so the image only has to be written once.

Sample Output
*************

The output below is a capture of one run, with an :zephyr:board:`nrf54lm20dk` running this
sample and an :zephyr:board:`nrf52840dk` running the ``smp_svr`` peer. Both consoles are
trimmed, and their timestamps have unrelated origins.

On the board running this sample, on a first boot with an unformatted external flash:

.. code-block:: console

   [00:18:10.564,967] <inf> spi_nor: mx25r6435f@0: 8 MiBy flash
   [00:18:10.567,247] <inf> littlefs: LittleFS version 2.11, disk version 2.1
   [00:18:10.567,361] <inf> littlefs: FS at mx25r6435f@0:0x0 is 256 0x1000-byte blocks with 512 cycle
   [00:18:10.567,368] <inf> littlefs: partition sizes: rd 16 ; pr 16 ; ca 64 ; la 32
   [00:18:10.567,834] <err> littlefs: lfs.c:1388: Corrupted dir pair at {0x0, 0x1}
   [00:18:10.567,841] <wrn> littlefs: can't mount (LFS -84); formatting
   [00:18:10.772,590] <inf> littlefs: /lfs1 mounted
   *** Booting Zephyr OS build 1d9804471afd ***
   [00:18:10.772,669] <inf> smp_bt_client_sample: MCUmgr SMP client Bluetooth sample
   [00:18:10.773,472] <inf> smp_bt_client_sample: No /lfs1/update.bin yet, writing a placeholder image
   [00:18:10.996,697] <inf> smp_bt_client_sample: Image /lfs1/update.bin is 4096 bytes
   [00:18:10.998,311] <inf> smp_bt_client_sample: Scanning for a peer that advertises the SMP service
   [00:18:11.077,547] <inf> smp_bt_client_sample: SMP server found at F1:63:F1:10:90:76 (random), RSSI -33
   [00:18:11.147,349] <inf> smp_bt_client_sample: Connected
   [00:18:11.848,733] <inf> mcumgr_smp: SMP client transport attached (value handle 0x000e, MTU 498)
   [00:18:11.848,742] <inf> smp_bt_client_sample: Transport attached to the peer's SMP service
   [00:18:11.948,875] <inf> smp_bt_client_sample: Peer answered the echo command
   [00:18:12.148,852] <inf> smp_bt_client_sample: Uploaded 1024/4096 bytes
   [00:18:12.248,853] <inf> smp_bt_client_sample: Uploaded 2048/4096 bytes
   [00:18:12.348,853] <inf> smp_bt_client_sample: Uploaded 3072/4096 bytes
   [00:18:12.448,853] <inf> smp_bt_client_sample: Uploaded 4096/4096 bytes
   [00:18:12.448,873] <inf> smp_bt_client_sample: Image written to the peer's secondary slot
   [00:18:12.499,228] <inf> smp_bt_client_sample: Disconnected (reason 0x16)

On a later boot the file is already there, so the run starts at ``Image /lfs1/update.bin is
4096 bytes``. The peer's console shows the other half of the same run:

.. code-block:: console

   [00:00:00.177,124] <inf> smp_bt_sample: Advertising successfully started
   [00:00:25.407,806] <inf> smp_bt_sample: Connected
   [00:00:26.759,765] <inf> smp_bt_sample: Disconnected, reason 0x13
   [00:00:26.760,375] <inf> smp_bt_sample: Advertising successfully started

The peer starts advertising again as soon as the client drops the link, so a second run
needs nothing done to it. Both disconnect reason codes are correct: the peer is told
``BT_HCI_ERR_REMOTE_USER_TERM_CONN`` (``0x13``) and the initiator
``BT_HCI_ERR_LOCALHOST_TERM_CONN`` (``0x16``).

The ``MTU 498`` in the attach line is worth checking on the first run: the transport writes
``ATT_MTU - 3`` byte fragments, so an MTU stuck at the default of 23 slows the transfer by an
order of magnitude. Only the GATT client can ask for a larger one, which is why this sample
sets :kconfig:option:`CONFIG_BT_GATT_AUTO_UPDATE_MTU`.

Diagnosing a failed run
=======================

Each stage fails with its own message, and the transport's own lines, logged under
``mcumgr_smp``, give the reason immediately before the sample reports the failed call.

``Nothing is mounted at /lfs1``
   There is no file system to read the image from, usually because the ``lfs1``
   :dtcompatible:`zephyr,fstab,littlefs` entry does not match the partition it points at.
   The file system subsystem logs the mount error just above.

``/lfs1/update.bin is N bytes, too small to be an image``
   The file exists but is shorter than an MCUboot image header. An interrupted upload can
   leave a zero length file behind; write it again.

``No SMP server to connect to``
   The 30 second scan expired. Check that the peer logged ``Advertising successfully
   started`` and was built with :file:`bt.conf`. If it is advertising and still not found,
   reset the client: the duplicate filter suppresses a peer after a failed attempt.

``Failed to attach the transport``
   Discovery, the subscription or the descriptor write failed, and the preceding
   ``mcumgr_smp`` line says which. A peer built without
   :kconfig:option:`CONFIG_MCUMGR_TRANSPORT_BT` has no SMP service; one built with
   :kconfig:option:`CONFIG_BT_SMP` defaults to
   :kconfig:option:`CONFIG_MCUMGR_TRANSPORT_BT_PERM_RW_AUTHEN` and refuses the subscription
   until the boards are paired.

``Echo command failed``
   The transport attached, so discovery and the subscription worked, but no SMP round trip
   completed. This isolates the failure to fragmentation and reassembly rather than GATT.

``Upload failed at offset 0 (err 6)``
   The peer's partition layout exposes no usable secondary slot. Error 1 at the same point
   instead means the file on :file:`/lfs1` does not start with the MCUboot image magic.

``Upload failed at offset N (err 4)``
   No answer arrived within :kconfig:option:`CONFIG_SMP_CMD_DEFAULT_LIFE_TIME`, so this is
   the client's own timeout rather than a verdict from the peer, which accepted the frames
   before it. The peer's console says what stalled it.
