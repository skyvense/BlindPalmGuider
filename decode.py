import codecs

data = bytes.fromhex('0D0A20B3C9B9A6A3BA0D0A4D6F64653D300D0A20B5E7D1B93D300D0A20B5E7C1F73D370D0A20C2F6B3E53D313030')

with open('E:/projects/PIO/superminis3/result.txt', 'w', encoding='utf-8') as f:
    f.write('HEX: ' + data.hex().upper() + '\n\n')
    f.write('GBK 解码:\n')
    f.write(data.decode('gbk', errors='replace'))

print('Done')