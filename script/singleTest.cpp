#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

class singleTest
{
public:
    int a;
};
struct data1
{
    int a;
    int b;
};

void default_val()
{
    int b;
    singleTest s;
    cout << s.a << endl;
    cout << b << endl;
}

void min_ele_lam()
{
    vector<data1> dd;
    data1 d1 = {1, 2};
    data1 d2 = {2, 2};
    data1 d3 = {0, 9};
    data1 d4 = {9, 0};
    dd.push_back(d1);
    dd.push_back(d2);
    dd.push_back(d3);
    dd.push_back(d4);
    data1 min_val = *min_element(dd.begin(), dd.end(), [](data1 a, data1 b)
                                 { return a.b < b.b; });
    cout << "min is:" << min_val.a << ", " << min_val.b << endl;
}

void copy_assign()
{
    vector<int> b;
    {
        vector<int> a{1, 2, 3, 4, 5};
        b = a;
    }
    cout << b[1] << endl;

    vector<int> c;
    for (int i = 0; i < 3; ++i)
    {
        vector<int> a{1, 2, 3, 4, 5};
        b.assign(a.begin(), a.end());
    }
    cout << b.size() << endl;
}

void reserve_test()
{
    vector<int> a;
    a.reserve(10);
    cout << a.size() << endl;
    cout << a.capacity() << endl;
    a.push_back(1);
    cout << a.size() << endl;
    cout << a.capacity() << endl;
}

void clear_test()
{
    vector<int> aa;
    cout << "1:" << aa.size() << endl;
    aa.resize(10);
    cout << "2:" << aa.size() << endl;
    fill(aa.begin(), aa.end(), 0);
    cout << "3:" << aa.size() << endl;
}

int main()
{
    // default_val();
    // min_ele_lam();
    // copy_assign();
    // clear_test();
    reserve_test();
}