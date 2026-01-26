#include <openssl/aes.h>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <sstream>
#include <cctype>


using namespace std;



// 十六进制字符串转换为字节向量
vector<unsigned char> from_hex_string(const string& hex_str) {
    vector<unsigned char> bytes;
    
    // 确保十六进制字符串长度为偶数
    if (hex_str.length() % 2 != 0) {
        throw runtime_error("十六进制字符串长度必须是偶数");
    }
    
    for (size_t i = 0; i < hex_str.length(); i += 2) {
        string byte_str = hex_str.substr(i, 2);
        
        // 验证十六进制字符
        for (char c : byte_str) {
            if (!isxdigit(c)) {
                throw runtime_error("无效的十六进制字符: " + string(1, c));
            }
        }
        
        unsigned int byte_value = stoul(byte_str, nullptr, 16);
        bytes.push_back(static_cast<unsigned char>(byte_value));
    }
    
    return bytes;
}


// 字节向量转换为十六进制字符串
string to_hex_string(const vector<unsigned char>& bytes) {
    stringstream ss;
    ss << hex << setfill('0');
    
    for (unsigned char byte : bytes) {
        ss << setw(2) << static_cast<int>(byte);
    }
    
    return ss.str();
}



vector<unsigned char> aes_128_cbc_encrypt(vector<unsigned char> &plain,
    vector<unsigned char> &key,
    vector<unsigned char> iv) {
    // 禁用警告
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"

    AES_KEY ctx;
    AES_set_encrypt_key(key.data(), 256, &ctx);
    vector<unsigned char> cipher(16);
    AES_cbc_encrypt(plain.data(), cipher.data(), 16, &ctx, iv.data(), AES_ENCRYPT);

    return cipher;
}

vector<unsigned char> aes_128_cbc_decrypt(vector<unsigned char> &cipher,
    vector<unsigned char> &key,
    vector<unsigned char> iv) {
    // 禁用警告
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    AES_KEY ctx;
    AES_set_decrypt_key(key.data(), 256, &ctx);
    vector<unsigned char> plain(16);
    AES_cbc_encrypt(cipher.data(), plain.data(), 16, &ctx, iv.data(), AES_DECRYPT);

    return plain;
}

int main() {
    vector<unsigned char> key = from_hex_string("2B7E151628AED2A6ABF7158809CF4F3C");
    vector<unsigned char> iv = from_hex_string("0001020304050607");
    vector<unsigned char> plain = from_hex_string("6BC1BEE22E409F96E93D7E117393172A");

    vector<unsigned char> cipher = aes_128_cbc_encrypt(plain, key, iv);

    cout << "plain  : " << to_hex_string(plain) << endl;
    cout << "cipher : " << to_hex_string(cipher) << endl;
    cout << "iv     : " << to_hex_string(iv) << endl;

    vector<unsigned char> decrypt_text = aes_128_cbc_decrypt(cipher, key, iv);

    cout << "decrypt: " << to_hex_string(decrypt_text) << endl;
    return 0;
}
