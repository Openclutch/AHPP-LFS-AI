All code relates to a game called Live For Speed, a realistic racing game. We are using a feature in the game called Insim that is a framework that we can send packets to the LFS server to do actions within it.

Project is laid out like this:
1. TougeRacing = AHPP Underground racing, we are usually developing this.
2. AI = AI for self driving cars, to be used within the server eventually.

When updating code, if it isn't commented already, include comments using standard commenting blocks above the function, so it can be read using a markdown tool.

I will manually change the version once there is enough in the changelog for an update.

Based on the version number update, include the changes within the CHANGELOG.md. always add it to the top, without a version number. i will add the version number manually and want to reduce merge conflicts. 

The project is based in .net 4.8, c# version 8, compiling for x86. We are using github actions to update the service automatically on merges from main.
