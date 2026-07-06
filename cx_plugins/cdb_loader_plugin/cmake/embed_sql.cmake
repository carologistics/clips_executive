if(NOT DEFINED INPUT)
message(FATAL_ERROR "INPUT is not set")
endif()

if(NOT DEFINED OUTPUT)
message(FATAL_ERROR "OUTPUT is not set")
endif()

file(READ "${INPUT}" SQL_CONTENT)

get_filename_component(OUTPUT_DIR "${OUTPUT}" DIRECTORY)
file(MAKE_DIRECTORY "${OUTPUT_DIR}")

file(WRITE "${OUTPUT}" "#pragma once\n")
file(APPEND "${OUTPUT}" "#include <string_view>\n\n")
file(APPEND "${OUTPUT}" "namespace cx {\n")
file(APPEND "${OUTPUT}" "inline constexpr std::string_view kViewSchemaSql = R\"sql(")
file(APPEND "${OUTPUT}" "${SQL_CONTENT}")
file(APPEND "${OUTPUT}" ")sql\";\n")
file(APPEND "${OUTPUT}" "}  // namespace cx\n")
