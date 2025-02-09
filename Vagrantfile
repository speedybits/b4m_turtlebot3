# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|
  # Base box: Ubuntu 22.04 LTS (Jammy Jellyfish)
  config.vm.box = "ubuntu/jammy64"
  
  # VM Provider Configuration
  config.vm.provider "virtualbox" do |vb|
    # Display the VirtualBox GUI
    vb.gui = true
    
    # Customize VM resources
    vb.memory = 16384  # 16GB RAM
    vb.cpus = 4
    
    # Enhanced graphics settings for Webots
    vb.customize ["modifyvm", :id, "--vram", "256"]  # Increased from 128MB
    vb.customize ["modifyvm", :id, "--graphicscontroller", "vboxsvga"]  # Changed from vmsvga
    vb.customize ["modifyvm", :id, "--accelerate3d", "on"]
    vb.customize ["modifyvm", :id, "--clipboard", "bidirectional"]
    vb.customize ["modifyvm", :id, "--draganddrop", "bidirectional"]
    vb.customize ["modifyvm", :id, "--vrde", "off"]
    vb.customize ["modifyvm", :id, "--pae", "on"]
    
    # Additional graphics performance settings
    vb.customize ["modifyvm", :id, "--hwvirtex", "on"]
    vb.customize ["modifyvm", :id, "--nestedpaging", "on"]
    vb.customize ["modifyvm", :id, "--largepages", "on"]
    
    # OpenGL settings
    vb.customize ["setextradata", :id, "CustomVideoMode1", "1920x1080x32"]
    vb.customize ["setextradata", :id, "GUI/MaxGuestResolution", "any"]
  end

  # Network Configuration
  config.vm.network "forwarded_port", guest: 8080, host: 8081

  # X11 forwarding
  config.ssh.forward_x11 = true
  config.ssh.forward_agent = true

  # Synced Folder Configuration
  config.vm.synced_folder ".", "/vagrant", disabled: true
  config.vm.synced_folder ".", "/home/vagrant/workspace"

  # Provisioning Scripts
  config.vm.provision "shell", path: "scripts/install_dependencies.sh"
  config.vm.provision "shell", path: "scripts/install_ros2.sh"
  config.vm.provision "shell", path: "scripts/install_webots.sh"
  config.vm.provision "shell", path: "scripts/setup_user.sh"
end
