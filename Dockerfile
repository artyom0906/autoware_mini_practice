FROM ubuntu:latest
LABEL authors="Artyom"

ENTRYPOINT ["top", "-b"]