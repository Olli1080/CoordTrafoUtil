vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO Olli1080/CoordTrafoUtil
    REF v0.1.1
    SHA512 a959fbede27a8e433c6a3e40b505090695f50bcdf44d2d12a50d441d72429f41deb71258f68edd10bf44288eebf368b7d1febce979dce21bc79a1b703abfa2cd
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
