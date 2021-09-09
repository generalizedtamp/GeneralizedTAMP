# Command to run: docker run --rm -v $(pwd):/GeneralizedTAMP -w /open-wbo/build <docker-repo-name> open-wbo
# Or add the following to your .profile/.bashrc/.zshrc

# open-wbo() {
#    docker run --rm -m=8g -v $(pwd):/GeneralizedTAMP -w /open-wbo/build <docker-repo-name> open-wbo $*
# }


FROM nvidia/cuda:10.1-cudnn7-devel-ubuntu18.04

# Note: we remove the files added by "apt-get update" in the same build command
# to avoid adding them to this layer (each command is a layer). If a file is
# added and removed in the same build command, they won't appear in the "diff"
# of the layer. This is optional but will keep the image as small as possible.
RUN apt-get update && apt-get install -y --no-install-recommends \
		 cmake \
		 libz-dev \
		 libgmp-dev 

# Copy your source code using paths relative to your project root. The second
# argument is the destination path within the container.
COPY . /open-wbo/

RUN cd /open-wbo && make

ENV PATH = "${PATH}:/open-wbo"

# ENTRYPOINT ["tail", "-f", "/dev/null"]