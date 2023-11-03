FROM ubuntu:22.04

RUN apt-get update && \
  apt-get install -y --no-install-recommends \
    git \
    maven \
    openjdk-17-jdk \
    libswt-gtk-4-jni libswt-gtk-4-java \
    fonts-noto-color-emoji \
  && rm -rf /var/lib/apt/lists/*

RUN git clone https://git.eclipse.org/r/tracecompass/org.eclipse.tracecompass.git
RUN cd org.eclipse.tracecompass && \
  git checkout 97835fcbc8dc9e6bd17b7666730798f9f4a0c92c && \
  mvn clean install \
    -DskipTests=true \
    -Dskip-rcp=true \
    -Dskip-tc-long-ui-tests=true \
    -Dskip-short-tc-ui-tests=true \
    -Dskip-tc-core-tests=true \
  && cd ..

RUN git clone https://git.eclipse.org/r/tracecompass.incubator/org.eclipse.tracecompass.incubator.git
RUN cd org.eclipse.tracecompass.incubator && \
  git checkout 16f16ba50ee5be38d85681db442e1cc78d89eee3 && \
  mvn clean install \
    -DskipTests=true \
  && cd ..

RUN cp org.eclipse.tracecompass.incubator/rcp/org.eclipse.tracecompass.incubator.rcp.product/target/products/trace-compass-*-linux.gtk.x86_64.tar.gz .
RUN mkdir tc && tar -xzf trace-compass-*-linux.gtk.x86_64.tar.gz -C tc

ENTRYPOINT ["/tc/trace-compass/tracecompass"]
CMD ["/bin/sh", "-c", "trap 'exit 147' TERM; tail -f /dev/null & while wait ${!}; [ $? -ge 128 ]; do true; done"]
