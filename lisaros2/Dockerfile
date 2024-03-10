FROM amazoncorretto:17.0.9-al2023-headful
RUN yum update && yum install -y unzip findutils graphviz
WORKDIR /lisaros2
COPY . .
RUN file=$(ls) && echo $file
RUN chmod 775 setup.sh && ./setup.sh
