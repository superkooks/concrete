idf_component_register(SRCS "main.cpp" "buttons.cpp" "lcd.cpp"
                    INCLUDE_DIRS ".")

find_package(nlohmann_json 3.12.0 REQUIRED)

target_link_libraries(${COMPONENT_LIB} PRIVATE nlohmann_json::nlohmann_json)
