#!/usr/bin/env bash
set -e
./gradlew clean
./gradlew build -x test -x javadoc
./gradlew distzip
unzip -o build/distributions/lisa4ros2-0.1a1.zip -d build/distributions
chmod 775 build/distributions/lisa4ros2-0.1a1/bin/lisa4ros2
ln -s -f build/distributions/lisa4ros2-0.1a1/bin/lisa4ros2 lisa4ros2