FROM cpp-base

RUN apt-get update \
	&& apt-get install -y --no-install-recommends clang-tidy clang-format libeigen3-dev \
	&& rm -rf /var/lib/apt/lists/*

ENV CC=clang
ENV CXX=clang++
