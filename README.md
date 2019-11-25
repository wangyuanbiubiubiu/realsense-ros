# realsense-ros for slam
Support t265, d435 and other cameras, as well as multi-camera mode


# Install
    cd <PATH TO YOUR CATKIN WS>/src
    git clone https://github.com/wangyuanbiubiubiu/realsense-ros.git
    cd ../
    catkin_make

# Run

    source <PATH TO YOUR CATKIN WS>/devel/setup.bash
    roslaunch myrealsense intel_d435i.launch



# Troubleshoot

**UVC header is not available**

    terminate called after throwing an instance of 'rs2::invalid_value_error'
      what():  UVC header is not available
      
Solution: [here](https://github.com/chutsu/rs4se/issues/3#issuecomment-530434550)


# LICENSE

Copyright (c) <2017> <Chris Choi>. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software must
display the following acknowledgement: This product includes software developed
by Chris Choi.

4. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

[librealsense2]: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
[install_prerequisit]: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md#prerequisites

