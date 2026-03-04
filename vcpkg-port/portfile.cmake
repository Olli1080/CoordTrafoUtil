vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO Olli1080/CoordTrafoUtil
    REF "v${VERSION}"
    SHA512 # TODO: Replace with actual SHA512 hash after tagging release
)

vcpkg_check_features(OUT_FEATURE_OPTIONS FEATURE_OPTIONS
    FEATURES
        eigen USE_EIGEN
        pcl   USE_PCL
        tests BUILD_TESTS
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        ${FEATURE_OPTIONS}
)

vcpkg_cmake_install()

vcpkg_cmake_config_fixup(
    PACKAGE_NAME base-transformation
    CONFIG_PATH share/base-transformation
)

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
