# Adds the controller image for the nRF5340 network core.  Its Kconfig
# comes from sysbuild/hci_ipc/prj.conf, which REPLACES the stock sample's
# prj.conf (sysbuild uses sysbuild/<image>/ as that image's config dir).

if(SB_CONFIG_NET_CORE_IMAGE_HCI_IPC)
  ExternalZephyrProject_Add(
    APPLICATION hci_ipc
    SOURCE_DIR  ${ZEPHYR_BASE}/samples/bluetooth/hci_ipc
    BOARD       ${SB_CONFIG_NET_CORE_BOARD}
  )
endif()
