FROM gitpod/workspace-full

USER root

# Install custom tools, runtime, etc.
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    && apt-get clean && rm -rf /var/cache/apt/* && rm -rf /var/lib/apt/lists/* && rm -rf /tmp/*