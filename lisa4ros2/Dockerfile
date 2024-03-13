FROM amazoncorretto:17.0.9-al2023-headful
RUN yum update && yum install -y unzip findutils graphviz
WORKDIR /lisa4ros2
COPY . .
RUN file=$(ls) && echo $file
RUN chmod 775 setup.sh && ./setup.sh
