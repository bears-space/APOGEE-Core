idf_component_register(
    SRCS
        "src/lsm6dsv.c"
        "vendor/lsm6dsv320x_reg.c"
    INCLUDE_DIRS
        "include"
        "vendor"
    PRIV_REQUIRES
        freertos
        vigilant_engine
)

# Vendored ST driver is not warning-clean under the project-wide -Werror.
set_source_files_properties(
    "${CMAKE_CURRENT_LIST_DIR}/vendor/lsm6dsv320x_reg.c"
    PROPERTIES COMPILE_OPTIONS "-Wno-error"
)
