///////////////////////////////////////////////////
//				date: 2025.03.09
//				author: ÁõÁ¢Ïò  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#ifndef REGISTER_H
#define REGISTER_H

class Register
{
public:
    Register();
    ~Register();
    bool isRegisted();
    bool showQQ();
    std::string getMark();

private:
    bool isRegisted(const char* code);

private:
    std::string code;
    bool registed;
};

#endif //REGISTER_H