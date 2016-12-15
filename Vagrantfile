# All Vagrant configuration is done below. The "2" in Vagrant.configure
# configures the configuration version (we support older styles for
# backwards compatibility). Please don't change it unless you know what
# you're doing.
Vagrant.configure("2") do |config|
  config.vm.box = "ubuntu/wily64"
 
  config.vm.provision "file", source: "./pixy_cross_compile.sh", destination: "pixy_cross_compile.sh"
  config.vm.synced_folder "frc_pixy/", "/home/vagrant/build/frc_pixy", create: true, owner: "vagrant", group: "vagrant"
end
