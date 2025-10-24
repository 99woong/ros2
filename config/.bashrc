# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

export ROS_DOMAIN_ID=11


source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash
alias cs='cd ~/ros2_ws/src/amr_emulator'
alias ab='cd ~/ros2_ws/src/amr_emulator/build'
alias fb='cd ~/ros2_ws/src/amr_emulator/test/fms_mqtt_publish/build'
alias cpf='cd ~/ros2_ws && colcon build --packages-select fms_mqtt_publish'
alias cpd='cd ~/ros2_ws && colcon build --packages-select driving_viewer'
alias rrff='ros2 run fms_mqtt_publish fms_mqtt_publish'
alias rrdd='ros2 run driving_viewer driving_viewer'
alias jetson='ssh nvidia@192.168.1.43'
alias makep='make -j$proc'
alias rrmm='ros2 run map_generator map_generator'
alias cbpg='cd ~/ros2_ws && colcon build --packages-select gls100_ros2'
alias rrgg='ros2 run gls100_ros2 gls100_node --ros-args -p can_device:=PCAN_USBBUS1 -p can_baudrate:=500000 -p node_id:=10'
alias cbp='colcon build --packages-select'
alias cbpv='cd ~/ros2_ws && cbp vloc_receiver'
alias cbpg='cd ~/ros2_ws && cbp gls100_ros2'
alias cbpp='cd ~/ros2_ws && cbp pose_monitor'
alias rlpp='ros2 launch pose_monitor pose_monitor.launch.py'
alias crsg='cd ~/ros2_ws/src && cp -rf ./vloc_receiver ./pose_monitor ./gls100_ros2 ~/git/ros2'
alias crsg='cd ~/git/ros2 && cp -rf ./vloc_receiver ./pose_monitor ./gls100_ros2 ~/ros2_ws/src'
alias gita='git add .'
alias gitc='git commit -m'
alias gsr='cd ~/git/ros2'
alias gsa='cd ~/git/amr_enulator'
alias gst='cd ~/git/test_amr_emulator'
alias cs='cd ~/ros2/src'



export LANGUAGE=ko_KR.UTF-8
export LANG=ko_KR.UTF-8
export ROS_DOMAIN_ID=11
