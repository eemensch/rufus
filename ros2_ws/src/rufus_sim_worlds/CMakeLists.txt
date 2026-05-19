cmake_minimum_required(VERSION 3.16)
project(rufus_sim_worlds)

find_package(ament_cmake REQUIRED)

set(_templates_dir ${CMAKE_CURRENT_SOURCE_DIR}/templates)
set(_manifests_dir ${CMAKE_CURRENT_SOURCE_DIR}/config/agents)
set(_generator ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_world.py)
set(_gen_out ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(_gen_stamp ${CMAKE_CURRENT_BINARY_DIR}/generated.stamp)

file(GLOB _template_files CONFIGURE_DEPENDS "${_templates_dir}/*.in")
file(GLOB _manifest_files CONFIGURE_DEPENDS "${_manifests_dir}/*.yaml")

# Each manifest in config/agents/ produces its own world SDF and
# its own set of per-instance model dirs in <build>/generated/.
# Run the generator once per manifest. The model dirs across
# manifests will collide if two manifests share the same
# (type, instance) pair — that's expected, and the YAML is
# authored so a given (type, instance) means the same vehicle.
set(_gen_cmds)
foreach(_m IN LISTS _manifest_files)
  list(APPEND _gen_cmds
    COMMAND python3 ${_generator}
            --manifest ${_m}
            --templates-dir ${_templates_dir}
            --out-dir ${_gen_out})
endforeach()

add_custom_command(
  OUTPUT ${_gen_stamp}
  COMMAND ${CMAKE_COMMAND} -E rm -rf ${_gen_out}
  ${_gen_cmds}
  COMMAND ${CMAKE_COMMAND} -E touch ${_gen_stamp}
  DEPENDS ${_manifest_files} ${_generator} ${_template_files}
  COMMENT "rufus_sim_worlds: generating per-instance models + worlds"
  VERBATIM)

add_custom_target(rufus_sim_worlds_generate ALL DEPENDS ${_gen_stamp})

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(
  DIRECTORY ${_gen_out}/worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(
  DIRECTORY ${_gen_out}/models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY ${_manifests_dir}
  DESTINATION share/${PROJECT_NAME}/config
)

ament_environment_hooks(env-hooks/rufus_sim_worlds.dsv.in)

ament_package()
