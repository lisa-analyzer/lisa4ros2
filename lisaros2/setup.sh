./gradlew clean
./gradlew build -x test -x javadoc
unzip -o build/distributions/lisaros2-0.1a1.zip -d build/distributions
chmod 775 build/distributions/lisaros2-0.1a1/bin/lisaros2
ln -s -f build/distributions/lisaros2-0.1a1/bin/lisaros2 lisaros2