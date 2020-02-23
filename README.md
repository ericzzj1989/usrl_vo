Building example
chmod +x build.sh
./build.sh


Execute the following command
Change KITTI.yaml to KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change dataset to the uncompressed dataset folder. Change sequences to 00, 01, 02,.., 11
./examples/stereo/stereo_kitti examples/stereo/KITTI.yaml dataset/sequences
