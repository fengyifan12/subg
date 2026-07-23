sdk_generate_library(ble_mesh_api)
sdk_add_include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/ble-mesh-api/include
)
sdk_library_add_sources(
    ${CMAKE_CURRENT_LIST_DIR}/ble-mesh-api/src/ble_mesh_lib_api.c
)


if((${CONFIG_CHIP} STREQUAL "RT582") OR (${CONFIG_CHIP} STREQUAL "RT581") OR (${CONFIG_CHIP} STREQUAL "RT583"))
    sdk_add_static_library_ifdef(CONFIG_SUPPORT_MESH_SERVER CM3_ble_mesh_server)
    sdk_add_static_library_ifdef(CONFIG_SUPPORT_MESH_CLIENT CM3_ble_mesh_client)
else((${CONFIG_CHIP} STREQUAL "RT584H") OR (${CONFIG_CHIP} STREQUAL "RT584HA4") OR (${CONFIG_CHIP} STREQUAL "RT584L"))
    sdk_add_static_library_ifdef(CONFIG_SUPPORT_MESH_SERVER CM33_ble_mesh_server)
    sdk_add_static_library_ifdef(CONFIG_SUPPORT_MESH_CLIENT CM33_ble_mesh_client)
endif()


sdk_add_include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/mib/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/pib/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/prod/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/profile/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/profile/beacon/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/profile/bearer/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/profile/config-model/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/profile/dsm/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/profile/friend/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/profile/lower-Layer/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/profile/lprn/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/profile/network/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/profile/provision/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/profile/proxy/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/profile/security/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/profile/security/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/profile/security/bearssl/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/profile/security/ecdh/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/profile/transport/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/profile/upper-layer/include
    ${CMAKE_CURRENT_LIST_DIR}/lib-ble-mesh/task/include
)
sdk_add_subdirectory_ifdef(CONFIG_BUILD_COMPONENT_BLUETOOTH_LE_MESH ${CMAKE_CURRENT_LIST_DIR}/model)