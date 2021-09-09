# Command to run: docker run --rm -v $(pwd):/GeneralizedTAMP -w /QNP2FOND/build <docker-repo-name> qnp2fond
# Or add the following to your .profile/.bashrc/.zshrc

# qnp2fond() {
#    docker run --rm -v $(pwd):/GeneralizedTAMP -w /QNP2FOND/build <docker-repo-name> qnp2fond $*
# }


FROM nvidia/cuda:10.1-cudnn7-devel-ubuntu18.04

# Note: we remove the files added by "apt-get update" in the same build command
# to avoid adding them to this layer (each command is a layer). If a file is
# added and removed in the same build command, they won't appear in the "diff"
# of the layer. This is optional but will keep the image as small as possible.
RUN apt-get update && apt-get install -y --no-install-recommends \
		 cmake

# Copy your source code using paths relative to your project root. The second
# argument is the destination path within the container.
COPY . /QNP2FOND/

RUN mkdir /QNP2FOND/build
RUN cd /QNP2FOND/build && cmake ..
RUN cd /QNP2FOND/build && make


ENV PATH = "${PATH}:/QNP2FOND/build"

# ENTRYPOINT ["tail", "-f", "/dev/null"]