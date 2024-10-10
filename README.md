# BTPG

## Usage

```bash
cmake -DCMAKE_BUILD_TYPE=RELEASE .
make
```
If you want to output more information, use the following command line.

```bash
cmake -DCMAKE_BUILD_TYPE=Debug .
make
```

```bash
./btpg -f Paris_1_256-random-10_150agents.txt -s 1 -a 1
```

The above command line will run BTPG-optimized on a MAPF plan with 150 agents on random scene-10 of benchmark map Paris. The relevant data (TPG and BTPG) are displayed directly in the terminal.

- -f: the MAPF plan file
- -s: seed
- -a: 0 for BTPG-naïve and 1 for BTPG-optimized

Also if you want to try other MAPF plans, there are other maps and scenarios to try in the `experiment/path` folder.


`experiment/path` 文件夹内包含很多mapf路径，可以尝试运行。