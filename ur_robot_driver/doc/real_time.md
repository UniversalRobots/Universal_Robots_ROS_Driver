# Setting up Ubuntu with a PREEMPT_RT kernel
In order to run the `universal_robot_driver`, we highly recommend to setup a ubuntu system with
real-time capabilities. Especially with a robot from the e-Series the higher control frequency
might lead to non-smooth trajectory execution if not run using a real-time-enabled system.

You might still be able to control the robot using a non-real-time system. This is, however, not recommended.

To get real-time support into a ubuntu system, the following steps have to be performed:
 1. Get the sources of a real-time kernel
 2. Compile the real-time kernel
 3. Setup user privileges to execute real-time tasks

This guide will help you setup your system with a real-time kernel.

## Preparing
To build the kernel, you will need a couple of tools available on your system. You can install them
using

``` bash
$ sudo apt-get install build-essential bc ca-certificates gnupg2 libssl-dev wget gawk flex bison
```

Before you download the sources of a real-time-enabled kernel, check the kernel version that is currently installed:

```bash
$ uname -r
4.15.0-62-generic 
```

To continue with this tutorial, please create a temporary folder and navigate into it. You should
have sufficient space (around 25GB) there, as the extracted kernel sources take much space. After
the new kernel is installed, you can delete this folder again.

In this example we will use a temporary folder inside our home folder:

```bash
$ mkdir -p ${HOME}/rt_kernel_build
$ cd ${HOME}/rt_kernel_build
```

All future commands are expected to be run inside this folder. If the folder is different, the `$`
sign will be prefixed with a path relative to the above folder.

## Getting the sources for a real-time kernel
To build a real-time kernel, we first need to get the kernel sources and the real-time patch.

First, we must decide on the kernel version that we want to use. Above, we
determined that our system has a 4.15 kernel installed. However, real-time
patches exist only for selected kernel versions. Those can be found on the
[linuxfoundation wiki](https://wiki.linuxfoundation.org/realtime/preempt_rt_versions).

In this example, we will select a 4.14 kernel. Select a kernel version close  to the
one installed on your system.

Go ahead and download the kernel sources, patch sources and their signature files:

```bash
$ wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/4.14/patch-4.14.139-rt66.patch.xz
$ wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/4.14/patch-4.14.139-rt66.patch.sign
$ wget https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.139.tar.xz
$ wget https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.139.tar.sign
```

To unzip the downloaded files do
```bash
$ xz -dk patch-4.14.139-rt66.patch.xz
$ xz -d linux-4.14.139.tar.xz
```

### Verification
Technically, you can skip this section, it is however highly recommended to verify the file
integrity of such a core component of your system!

To verify file integrity, you must first import public keys by the kernel developers and the patch
author. For the kernel sources use (as suggested on
[kernel.org](https://www.kernel.org/signature.html))

```bash
$ gpg2 --locate-keys torvalds@kernel.org gregkh@kernel.org
```

and for the patch search for a key of the author listed on 
[linuxfoundation wiki](https://wiki.linuxfoundation.org/realtime/preempt_rt_versions).

```bash
$ gpg2 --keyserver hkp://keys.gnupg.net --search-keys zanussi
gpg: data source: http://51.38.91.189:11371
(1)     German Daniel Zanussi <german.zanussi@globant.com>
          4096 bit RSA key 0x537F98A9D92CEAC8, created: 2019-07-24, expires: 2023-07-24
(2)     Michael Zanussi <mzanussi@gmail.com>
          4096 bit RSA key 0x7C7F76A2C1E3D9EB, created: 2019-05-08
(3)     Tom Zanussi <tzanussi@gmail.com>
        Tom Zanussi <zanussi@kernel.org>
        Tom Zanussi <tom.zanussi@linux.intel.com>
          4096 bit RSA key 0xDE09826778A38521, created: 2017-12-15
(4)     Riccardo Zanussi <riccardo.zanussi@gmail.com>
          2048 bit RSA key 0xD299A06261D919C3, created: 2014-08-27, expires: 2018-08-27 (expired)
(5)     Zanussi Gianni <g.zanussi@virgilio.it>
          1024 bit DSA key 0x78B89CB020D1836C, created: 2004-04-06
(6)     Michael Zanussi <zanussi@unm.edu>
        Michael Zanussi <mzanussi@gmail.com>
        Michael Zanussi <michael_zanussi@yahoo.com>
        Michael Zanussi <michael@michaelzanussi.com>
          1024 bit DSA key 0xB3E952DCAC653064, created: 2000-09-05
(7)     Michael Zanussi <surfpnk@yahoo.com>
          1024 bit DSA key 0xEB10BBD9BA749318, created: 1999-05-31
(8)     Michael B. Zanussi <surfpnk@yahoo.com>
          1024 bit DSA key 0x39EE4EAD7BBB1E43, created: 1998-07-16
Keys 1-8 of 8 for "zanussi".  Enter number(s), N)ext, or Q)uit > 3
```

Now we can verify the downloaded sources:
```bash
$ gpg2 --verify linux-4.14.139.tar.sign
gpg: assuming signed data in 'linux-4.14.139.tar'
gpg: Signature made Fr 16 Aug 2019 10:15:17 CEST
gpg:                using RSA key 647F28654894E3BD457199BE38DBBDC86092693E
gpg: Good signature from "Greg Kroah-Hartman <gregkh@kernel.org>" [unknown]
gpg: WARNING: This key is not certified with a trusted signature!
gpg:          There is no indication that the signature belongs to the owner.
Primary key fingerprint: 647F 2865 4894 E3BD 4571  99BE 38DB BDC8 6092 693E

$ gpg2 --verify patch-4.14.139-rt66.patch.sign
gpg: assuming signed data in 'patch-4.14.139-rt66.patch'
gpg: Signature made Fr 23 Aug 2019 21:09:20 CEST
gpg:                using RSA key 0x0129F38552C38DF1
gpg: Good signature from "Tom Zanussi <tom.zanussi@linux.intel.com>" [unknown]
gpg:                 aka "Tom Zanussi <zanussi@kernel.org>" [unknown]
gpg:                 aka "Tom Zanussi <tzanussi@gmail.com>" [unknown]
gpg: WARNING: This key is not certified with a trusted signature!
gpg:          There is no indication that the signature belongs to the owner.
Primary key fingerprint: 5BDF C45C 2ECC 5387 D50C  E5EF DE09 8267 78A3 8521
     Subkey fingerprint: ACF8 5F98 16A8 D5F0 96AE  1FD2 0129 F385 52C3 8DF1
```

## Compilation
Before we can compile the sources, we have to extract the tar archive and apply the patch

```bash
$ tar xf linux-4.14.139.tar
$ cd linux-4.14.139
linux-4.14.139$ xzcat ../patch-4.14.139-rt66.patch.xz | patch -p1 
```

Now to configure your kernel, just type
```bash
linux-4.14.139$ make oldconfig
```

This will ask for kernel options. For everything else then the `Preemption Model` use the default
value (just press Enter) or adapt to your preferences. For the preemption model select `Fully Preemptible Kernel`:

```bash
Preemption Model
  1. No Forced Preemption (Server) (PREEMPT_NONE)
> 2. Voluntary Kernel Preemption (Desktop) (PREEMPT_VOLUNTARY)
  3. Preemptible Kernel (Low-Latency Desktop) (PREEMPT__LL) (NEW)
  4. Preemptible Kernel (Basic RT) (PREEMPT_RTB) (NEW)
  5. Fully Preemptible Kernel (RT) (PREEMPT_RT_FULL) (NEW)
choice[1-5]: 5
```

Now you can build the kernel. This will take some time...

```bash
linux-4.14.139$ make -j `getconf _NPROCESSORS_ONLN` deb-pkg
```

After building, install the `linux-headers` and `linux-image` packages in the parent folder (only
the ones without the `-dbg` in the name)

```bash
linux-4.14.139$ sudo apt install ../linux-headers-4.14.139-rt66_*.deb ../linux-image-4.14.139-rt66_*.deb
```

## Setup user privileges to use real-time scheduling
To be able to schedule threads with user privileges (what the driver will do) you'll have to change
the user's limits by changing `/etc/security/limits.conf` (See [the manpage](https://manpages.ubuntu.com/manpages/bionic/man5/limits.conf.5.html) for details)

We recommend to setup a group for real-time users instead of writing a fixed username into the config
file:

```bash
$ sudo groupadd realtime
$ sudo usermod -aG realtime $(whoami)
```

Then, make sure `/etc/security/limits.conf` contains
```
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
```

Note: You will have to log out and log back in (Not only close your terminal window) for these
changes to take effect. No need to do this now, as we will reboot later on, anyway.

## Setup GRUB to always boot the real-time kernel
To make the new kernel the default kernel that the system will boot into every time, you'll have to
change the grub config file inside `/etc/default/grub`.

Note: This works for ubuntu, but might not be working for other linux systems. It might be necessary
to use another menuentry name there.

But first, let's find out the name of the entry that we will want to make the default. You can list
all available kernels using

```bash
$ awk -F\' '/menuentry |submenu / {print $1 $2}' /boot/grub/grub.cfg

menuentry Ubuntu
submenu Advanced options for Ubuntu
    menuentry Ubuntu, with Linux 4.15.0-62-generic
    menuentry Ubuntu, with Linux 4.15.0-62-generic (recovery mode)
    menuentry Ubuntu, with Linux 4.15.0-60-generic
    menuentry Ubuntu, with Linux 4.15.0-60-generic (recovery mode)
    menuentry Ubuntu, with Linux 4.15.0-58-generic
    menuentry Ubuntu, with Linux 4.15.0-58-generic (recovery mode)
    menuentry Ubuntu, with Linux 4.14.139-rt66
    menuentry Ubuntu, with Linux 4.14.139-rt66 (recovery mode)
menuentry Memory test (memtest86+)
menuentry Memory test (memtest86+, serial console 115200)
menuentry Windows 7 (on /dev/sdc2)
menuentry Windows 7 (on /dev/sdc3)
```

From the output above, we'll need to generate a string with the pattern `"submenu_name>entry_name"`. In our case this would be

```
"Advanced options for Ubuntu>Ubuntu, with Linux 4.14.139-rt66"
```
**The double quotes and no spaces around the `>` are important!**

With this, we can setup the default grub entry and then update the grub menu entries. Don't forget this last step!

```bash
$ sudo sed -i 's/^GRUB_DEFAULT=.*/GRUB_DEFAULT="Advanced options for Ubuntu>Ubuntu, with Linux 4.14.139-rt66"/' /etc/default/grub
$ sudo update-grub
```

## Reboot the PC
After having performed the above mentioned steps, reboot the PC. It should boot into the correct
kernel automatically.

## Check for preemption capabilities
Make sure that the kernel does indeed support real-time scheduling:

```bash
$ uname -v | cut -d" " -f1-4 
#1 SMP PREEMPT RT
```

## Optional: Disable CPU speed scaling
Many modern CPUs support changing their clock frequency dynamically depending on the currently
requested computation resources. In some cases this can lead to small interruptions in execution.
While the real-time scheduled controller thread should be unaffected by this, any external
components such as a visual servoing system might be interrupted for a short period on scaling
changes.

To check and modify the power saving mode, install cpufrequtils:
```bash
$ sudo apt install cpufrequtils
```

Run `cpufreq-info` to check available "governors" and the current CPU Frequency (`current CPU
frequency is XXX MHZ`). In the following we will set the governor to "performance".

```bash
$ sudo systemctl disable ondemand
$ sudo systemctl enable cpufrequtils
$ sudo sh -c 'echo "GOVERNOR=performance" > /etc/default/cpufrequtils'
$ sudo systemctl daemon-reload && sudo systemctl restart cpufrequtils
```

This disables the `ondemand` CPU scaling daemon, creates a `cpufrequtils` config file and restarts
the `cpufrequtils` service. Check with `cpufreq-info`.

For further information about governors, please see the [kernel
documentation](https://www.kernel.org/doc/Documentation/cpu-freq/governors.txt).
