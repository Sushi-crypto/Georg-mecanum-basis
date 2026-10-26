# 1. Basis: Ein sauberes, stabiles Linux-Fundament
FROM ubuntu:22.04

# 2. Verhindert, dass der Bauprozess nach Tastatureingaben fragt
ENV DEBIAN_FRONEND=noninteractive

# 3. Werkzeugkiste packen: C++ Compiler, CMake, Git und CppUTest installieren
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    cpputest \
    pkg-config\
    g++ \
    gcc \
    && rm -rf /var/lib/apt/lists/*

# 4. Das Arbeitsverzeichnis im Container festlegen
WORKDIR /workspace

# 5. Standardbefehl beim Starten des Containers
CMD ["bash"]
