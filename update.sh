#!/bin/bash
./gradlew :spotlessApply
./gradlew publish
cp -rf build/repos/releases/YourLibTemplate YourLibTemplate/repos/
