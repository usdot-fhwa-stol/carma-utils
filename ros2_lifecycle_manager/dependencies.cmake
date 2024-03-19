find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # TODO(CAR-6016): Integrate addional tests into package
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_copyright
    ament_cmake_cppcheck
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_lint_cmake
    ament_cmake_pep257
    ament_cmake_xmllint
  )

  ament_lint_auto_find_test_dependencies()
endif()
