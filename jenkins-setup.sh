# https://pkg.jenkins.io/debian-stable/
# Install java
sudo apt -y install default-jre
# Add the jenkins package repository
wget -q -O - https://pkg.jenkins.io/debian-stable/jenkins.io.key | sudo apt-key add -
# https://stackoverflow.com/questions/3557037
SOURCE_NAME="deb https://pkg.jenkins.io/debian-stable binary/"
FILE_PATH="/etc/apt/sources.list"
sudo grep -qxF "$SOURCE_NAME" "$FILE_PATH" || echo "$SOURCE_NAME" | sudo tee --append "$FILE_PATH"
# install jenkins
sudo apt-get -y update
sudo apt-get -y install jenkins


