def convert_hex_to_desired_format(input_file, output_file):
    with open(input_file, 'r') as file:
        lines = file.readlines()

    output_lines = []

    id = 0

    for line in lines:
        if line.startswith(':'):
            data = line[9:-2] 
            bytes = [int(data[i:i+2], 16) for i in range(0, len(data), 2)]
            for byte in bytes:
                output_lines.append(f"{byte},   // id: {id}")
                id += 1

    with open(output_file, 'w') as file:
        file.write('\n'.join(output_lines))

convert_hex_to_desired_format('input.hex', 'output.txt')

# python read_hex.py .pio/build/EEP/firmware.eep  output.txt