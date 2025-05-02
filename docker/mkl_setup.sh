#!/bin/sh

# perform a binary install on x86 machines
# on ARM, it is skipped

if [ "${HW_ARCH}" = "x86_64" ]; then
  cd /root
  apt-get update && apt-get install -y gpg-agent wget
  wget -O- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | gpg --dearmor | sudo tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null
  echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" | sudo tee /etc/apt/sources.list.d/oneAPI.list
  apt-get update && apt-get install -y intel-oneapi-mkl
  echo 'source /opt/intel/oneapi/setvars.sh intel64' >> ~/.bashrc
  
elif [ "${HW_ARCH}" = "aarch64" ]; then
  echo 'Skipping MKL on ARM'
fi


exit 0
