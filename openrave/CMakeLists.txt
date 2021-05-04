# Copyright: 2017 UC3M
# Author: Raúl de Santos Rico
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname openrave)

find_package(YARP REQUIRED)
yarp_configure_external_installation(teo-openrave-models)

#yarp_install(FILES ymanager.ini DESTINATION ${TEO_OPENRAVE_MODELS_DATA_INSTALL_DIR})  # Cleaner than ${TEO_OPENRAVE_MODELS_ROBOTS_INSTALL_DIR}/../ or similar.

file(GLOB teo ${CMAKE_CURRENT_SOURCE_DIR}/teo/*.xml)
file(GLOB teo/models ${CMAKE_CURRENT_SOURCE_DIR}/teo/models/*.wrl)
#file(GLOB teo/ik ${CMAKE_CURRENT_SOURCE_DIR}/teo/ik/*.so)

yarp_install(FILES ${teo} DESTINATION ${TEO-OPENRAVE-MODELS_CONTEXTS_INSTALL_DIR}/${appname}/teo)
yarp_install(FILES ${teo/models} DESTINATION ${TEO-OPENRAVE-MODELS_CONTEXTS_INSTALL_DIR}/${appname}/teo/models)
#yarp_install(FILES ${teo/ik} DESTINATION ${TEO-OPENRAVE-MODELS_CONTEXTS_INSTALL_DIR}/${appname}/teo/ik)
