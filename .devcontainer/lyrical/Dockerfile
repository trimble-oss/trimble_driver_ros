FROM ros:lyrical
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip sudo
ENV SHELL /bin/bash

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/nopasswd_user \
    && chmod 0440 /etc/sudoers.d/nopasswd_user

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

RUN apt-get install -y clang-format python3-colcon-common-extensions python3-colcon-cd

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]