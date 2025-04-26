import xml.etree.ElementTree as ET

def extract_data_from_xml(xml_file):
    # 解析XML文件
    tree = ET.parse(xml_file)
    root = tree.getroot()

    data_dict = {}

    # 遍历每个interval标签
    for interval in root.findall('interval'):
        # 提取begin、end和id属性
        begin = interval.get('begin')
        end = interval.get('end')
        id_ = interval.get('id')

        # 构建键，格式为(begin, end)
        key = (begin, end)

        # 提取flow数据
        flow = float(interval.get('flow'))

        # 提取ID前缀
        id_prefix = id_.split('_')[0]

        # 创建或获取对应键的列表
        if key not in data_dict:
            data_dict[key] = {}

        # 添加数据到对应键的字典中
        if id_prefix not in data_dict[key]:
            data_dict[key][id_prefix] = 0
        data_dict[key][id_prefix] += flow

    return data_dict

def save_data_to_txt(data_dict, output_file):
    with open(output_file, 'w') as f:
        for key, id_prefix_data in data_dict.items():
            f.write(f"Data for interval ({key[0]}, {key[1]}):\n")
            for id_prefix, flow_sum in id_prefix_data.items():
                f.write(f"ID Prefix: {id_prefix}, Flow Sum: {flow_sum}\n")
            f.write('\n')


# XML文件路径
xml_file = 'output_E1'

# 提取数据并按(begin, end)分组
data_dict = extract_data_from_xml(xml_file)

E0_throughput = []
E2_throughput = []

# 收集吞吐量信息
for key, id_data in data_dict.items():
    for id, flow in id_data.items():
        if id == 'E0':
            E0_throughput.append(flow)
        else:
            E2_throughput.append(flow)

# 打印分组后的数据
for key, id_prefix_data in data_dict.items():
    print(f"Data for interval ({key[0]}, {key[1]}):")
    for id_prefix, flow_sum in id_prefix_data.items():
        print(f"ID Prefix: {id_prefix}, Flow Sum: {flow_sum}")
    print()

print(data_dict)

E0_throughput.sort()
E2_throughput.sort()

E0 = E0_throughput
E2 = E2_throughput

print(E0, E2)
E0_total = 0
E2_total = 0

for i in E0:
    E0_total += i
for j in E2:
    E2_total += j

print(E0_total, E2_total)
print()
print('Throughput in: ', E0_total / len(E0),'Bottleneck throughput: ', E2_total / len(E2))

# output_file = 'data_output_1800.txt'
# save_data_to_txt(data_dict, output_file)
