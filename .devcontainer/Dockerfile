FROM cpp-base

RUN apt-get update \
	&& apt-get install -y --no-install-recommends clang-tidy clang-format libeigen3-dev \
		python3-matplotlib python3-tk \
	&& rm -rf /var/lib/apt/lists/*

ENV CC=clang
ENV CXX=clang++
