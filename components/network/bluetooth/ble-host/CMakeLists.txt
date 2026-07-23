if((${CONFIG_CHIP} STREQUAL "RT582") OR (${CONFIG_CHIP} STREQUAL "RT581") OR (${CONFIG_CHIP} STREQUAL "RT583"))
     sdk_add_static_library(CM3_ble_host)
elseif((${CONFIG_CHIP} STREQUAL "RT584H") OR (${CONFIG_CHIP} STREQUAL "RT584HA4") OR (${CONFIG_CHIP} STREQUAL "RT584L"))
    sdk_add_static_library(CM33_ble_host)
endif()

sdk_add_include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/ble-cmd-api/include
    ${CMAKE_CURRENT_LIST_DIR}/hci/include
    ${CMAKE_CURRENT_LIST_DIR}/fota/include
    ${CMAKE_CURRENT_LIST_DIR}/uecc
)
sdk_add_subdirectory_ifdef(CONFIG_BUILD_COMPONENT_BLUETOOTH_LE_HOST ${CMAKE_CURRENT_LIST_DIR}/ble-cmd-api)
sdk_add_subdirectory_ifdef(CONFIG_BUILD_COMPONENT_BLUETOOTH_LE_HOST ${CMAKE_CURRENT_LIST_DIR}/ble-service)
sdk_add_subdirectory_ifdef(CONFIG_BLUETOOTH_LE_SERVICE_FOTA ${CMAKE_CURRENT_LIST_DIR}/fota)