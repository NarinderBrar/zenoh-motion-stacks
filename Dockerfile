FROM ubuntu:24.04

WORKDIR /src/motion-stacks
COPY . .
RUN scripts/install_dependencies.sh --yes
RUN cmake --preset system -DZMS_ENABLE_LIVE_TESTS=ON && \
    cmake --build --preset system -j 2 && \
    ctest --preset system

CMD ["bash"]
