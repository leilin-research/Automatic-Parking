#!/bin/bash

LX_PACKAGES_PATH=~/packages.lingxian.com/ros/ubuntu
LX_DEB_FILES_PATH=${LX_PACKAGES_PATH}/pool/main/
LX_PKG_AMD64_FILES_PATH=${LX_PACKAGES_PATH}/dists/trusty/main/binary-amd64

mkdir -p ${LX_DEB_FILES_PATH}
mkdir -p ${LX_PKG_I386_FILES_PATH}
mkdir -p ${LX_PKG_AMD64_FILES_PATH}

pushd install
rm -rf extract/
mkdir -p extract/DEBIAN
mkdir -p extract/opt/ros/indigo
cp ../control extract/DEBIAN/
cp -a share extract/opt/ros/indigo/
cp -a lib extract/opt/ros/indigo/
dpkg-deb -b extract/ .
cp *.deb ${LX_DEB_FILES_PATH}/

pushd ${LX_PACKAGES_PATH}
dpkg-scanpackages pool/main /dev/null | gzip > ${LX_PKG_AMD64_FILES_PATH}/Packages.gz
popd

popd
