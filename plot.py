import matplotlib.pyplot as plt

# 读取txt文件并按行解析成数字列表
def read_numbers_from_file(filename):
    numbers = []
    with open(filename, 'r') as file:
        for line in file:
            try:
                # 将每行转换为浮点数，并加入列表
                numbers.append(float(line.strip()))
            except ValueError:
                print(f"Warning: Ignoring non-numeric line: {line.strip()}")
    return numbers

# 绘制示意图
def plot_numbers(numbers):
    plt.figure(figsize=(10, 6))
    plt.scatter(range(len(numbers)), numbers, color='b', label='Data Points')
    plt.title('Number Plot from txt File')
    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.legend()
    plt.grid(True)
    plt.show()

# 主程序
if __name__ == "__main__":
    filename = 'output.txt'  # txt文件的路径
    numbers = read_numbers_from_file(filename)
    if numbers:
        plot_numbers(numbers)
    else:
        print("No valid numbers found in the file.")
