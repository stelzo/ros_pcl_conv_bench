.PHONY: all

all: run

build:
	@echo "Building CMake project..."
	@colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
	@echo "Building Rust project..."
	@cd rpcl2 && cargo build --release && cd ..

clean:
	@echo "Cleaning project..."
	@colcon clean workspace -y
	@echo "Cleaning Rust project..."
	@cd rpcl2 && cargo clean && cd ..
	@rm -rf results

run: build
	@echo "Running C++ with PCL...\n\n"
	@rm -rf results && mkdir results
	@rm -rf images && mkdir images
	@echo "\nMove data...\n"
	@hyperfine --warmup 3 ./build/pcl/roundtrip_16k_pcl ./rpcl2/target/release/roundtrip_vec_16k ./rpcl2/target/release/roundtrip_16k --export-markdown=./results/roundtrip_vec_16k.md --export-json=./results/roundtrip_16k.json
	@echo "\n—————————————\n"
	@hyperfine --warmup 3 ./build/pcl/roundtrip_60k_pcl ./rpcl2/target/release/roundtrip_vec_60k ./rpcl2/target/release/roundtrip_60k --export-markdown=./results/roundtrip_vec_60k.md --export-json=./results/roundtrip_60k.json
	@echo "\n—————————————\n"
	@hyperfine --warmup 3 ./build/pcl/roundtrip_120k_pcl ./rpcl2/target/release/roundtrip_vec_120k ./rpcl2/target/release/roundtrip_120k --export-markdown=./results/roundtrip_vec_120k.md --export-json=./results/roundtrip_120k.json
	@echo "\n—————————————\n"
	@hyperfine --warmup 3 ./build/pcl/roundtrip_500k_pcl ./rpcl2/target/release/roundtrip_vec_500k ./rpcl2/target/release/roundtrip_500k --export-markdown=./results/roundtrip_vec_500k.md --export-json=./results/roundtrip_500k.json
	@echo "\n—————————————\n"
	@hyperfine --warmup 3 ./build/pcl/roundtrip_1_5m_pcl ./rpcl2/target/release/roundtrip_vec_1_5m ./rpcl2/target/release/roundtrip_1_5m --export-markdown=./results/roundtrip_vec_1_5m.md --export-json=./results/roundtrip_1_5m.json
	@echo "\n—————————————\n"
	@echo "\nFilter x range per point...\n"
	@hyperfine --warmup 3 ./build/pcl/filter_16k_pcl ./build/pcl/filter_par_16k_pcl ./rpcl2/target/release/filter_par_par_16k ./rpcl2/target/release/filter_vec_16k ./rpcl2/target/release/filter_16k --export-markdown=./results/filter_vec_16k.md --export-json=./results/filter_16k.json
	@echo "\n—————————————\n"
	@hyperfine --warmup 3 ./build/pcl/filter_60k_pcl ./build/pcl/filter_par_60k_pcl ./rpcl2/target/release/filter_par_par_60k ./rpcl2/target/release/filter_vec_60k ./rpcl2/target/release/filter_60k --export-markdown=./results/filter_vec_60k.md --export-json=./results/filter_60k.json
	@echo "\n—————————————\n"
	@hyperfine --warmup 3 ./build/pcl/filter_120k_pcl ./build/pcl/filter_par_120k_pcl ./rpcl2/target/release/filter_par_par_120k ./rpcl2/target/release/filter_vec_120k ./rpcl2/target/release/filter_120k --export-markdown=./results/filter_vec_120k.md --export-json=./results/filter_120k.json
	@echo "\n—————————————\n"
	@hyperfine --warmup 3 ./build/pcl/filter_500k_pcl ./build/pcl/filter_par_500k_pcl ./rpcl2/target/release/filter_par_par_500k ./rpcl2/target/release/filter_vec_500k ./rpcl2/target/release/filter_500k --export-markdown=./results/filter_vec_500k.md --export-json=./results/filter_500k.json
	@echo "\n—————————————\n"
	@hyperfine --warmup 3 ./build/pcl/filter_1_5m_pcl ./build/pcl/filter_par_1_5m_pcl ./rpcl2/target/release/filter_par_par_1_5m ./rpcl2/target/release/filter_vec_1_5m ./rpcl2/target/release/filter_1_5m --export-markdown=./results/filter_vec_1_5m.md --export-json=./results/filter_1_5m.json
	@echo "\n—————————————\n"
	@echo "\nHeavy compute per point...\n"
	@hyperfine --warmup 3 ./build/pcl/compute_16k_pcl ./build/pcl/compute_par_16k_pcl ./rpcl2/target/release/compute_par_par_16k ./rpcl2/target/release/compute_16k --export-markdown=./results/compute_16k.md --export-json=./results/compute_16k.json
	@echo "\n—————————————\n"
	@hyperfine --warmup 3 ./build/pcl/compute_60k_pcl ./build/pcl/compute_par_60k_pcl ./rpcl2/target/release/compute_par_par_60k ./rpcl2/target/release/compute_60k --export-markdown=./results/compute_60k.md --export-json=./results/compute_60k.json
	@echo "\n—————————————\n"
	@hyperfine --warmup 3 ./build/pcl/compute_par_120k_pcl ./rpcl2/target/release/compute_par_par_120k --export-markdown=./results/compute_par_par_120k.md --export-json=./results/compute_par_par_120k.json
	@echo "\n—————————————\n"
	@hyperfine --warmup 3 ./build/pcl/compute_par_500k_pcl ./rpcl2/target/release/compute_par_par_500k --export-markdown=./results/compute_par_par_500k.md --export-json=./results/compute_par_par_500k.json
	@echo "\n—————————————\n"
	@hyperfine --warmup 3 ./build/pcl/compute_par_1_5m_pcl ./rpcl2/target/release/compute_par_par_1_5m --export-markdown=./results/compute_par_par_1_5m.md --export-json=./results/compute_par_par_1_5m.json
	@echo "\n—————————————\n"
	@echo "... finished."
	@./populate.sh



