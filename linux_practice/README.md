mkdir -p ~/yopo_training/linux_practice
cd ~/yopo_training/linux_practice
mkdir code data results reports problem_logs
touch README.md
pwd
tree
nano code/test_ubuntu.py
python3 code/test_ubuntu.py
cp data/sample.txt results/
mv results/sample.txt results/sample_new.txt
ls -l code/
touch temp.txt
rm temp.txt

本次练习工作目录: ~/yopo_training/linux_practice
mkdir -p ~/yopo_training/linux_practice/{code.data,results,reports,problems_logs}
cd ~/yopo_training/linux_practice
python3 test_ubuntu.py
