## C++

这部分主要涉及到整个slam系统中涉及到的C++编程技巧。



### 1. 智能指针

通常在写一个类的时候，我们会设置这个类的智能指针。

举例 Camera类，这里将智能指针定义为Camera的指针类型：

`typedef std::shared_ptr<Camera> Ptr;`

传参数的时候，使用Camera::Ptr即可：

`myslam::Camera::Ptr camera (new myslam::Camera);`

#### 详解

智能指针主要目的是方便c++的内存管理。智能指针就是一个类，当超出了类的作用域，类会自动调用析构函数，析构函数会自动释放资源。

#### 实例



##### test类

```c++
#include<iostream>
#include<memory>

using namespace std;

// 简简单单的一个类
class Test {
public:
    Test(string s) {
        str = s;
        cout << "Test creat\n";
    }

    ~Test() {
        cout << "Test delete:" << str << endl;
    }
    // 成员函数
    string &getStr() {
        return str;
    }

    void setStr(string s) {
        str = s;
    }

    void print() {
        cout << str << endl;
    }

private:
    string str;
};

// 一个公有函数
unique_ptr<Test> fun() {
    return unique_ptr<Test>(new Test("789"));//调用了构造函数，输出Test creat
}
```





##### a. shared_ptr

该类指针实现资源被多个指针共享，使用计数机制。通过成员函数use_count()查看资源共享个数。可以通过new、传入auto_ptr、unique_ptr、weak_ptr来构造。调用realease()时，指针会释放资源所有权，count-1。当count==0，资源被释放。

```c++
int main()
{
    shared_ptr<Test> ptest(new Test("123"));//调用构造函数输出Test create
    shared_ptr<Test> ptest2(new Test("456"));//调用构造函数输出 Test creat
    cout<<ptest2->getStr()<<endl;//输出456
    cout<<ptest2.use_count()<<endl;//显示此时资源被几个指针共享，输出1
    ptest = ptest2;//"456"引用次数加1，“123”销毁，输出Test delete：123
    ptest->print();//输出456
    cout<<ptest2.use_count()<<endl;//该指针指向的资源被几个指针共享，输出2
    cout<<ptest.use_count()<<endl;//2
    ptest.reset();//重新绑定对象，绑定一个空对象，当时此时指针指向的对象还有其他指针能指向就不会释放该对象的内存空间，
    ptest2.reset();//此时“456”销毁，此时指针指向的内存空间上的指针为0，就释放了该内存，输出Test delete
    cout<<"done !\n";
    return 0;
}
```

##### b. weak_ptr

weak_ptr是用来解决shared_ptr相互引用时的死锁问题,如果说两个shared_ptr相互引用,那么这两个指针的引用计数永远不可能下降为0,资源永远不会释放。它是对对象的一种弱引用，不会增加对象的引用计数，和shared_ptr之间可以相互转化，shared_ptr可以直接赋值给它，它可以通过调用lock函数来获得shared_ptr。



错误使用的时候，会出现如下情况：

```C++
#include<iostream>
#include<memory>
using namespace std;
class B;
class A
{
public:
    shared_ptr<B> pb_;
    ~A()
    {
        cout<<"A delete\n";
    }
};
class B
{
public:
    shared_ptr<A> pa_;
    ~B()
    {
        cout<<"B delete\n";
    }
};

void fun()
{
    shared_ptr<B> pb(new B());
    shared_ptr<A> pa(new A());
    pb->pa_ = pa;
    pa->pb_ = pb;
    cout<<pb.use_count()<<endl;
    cout<<pa.use_count()<<endl;
}

int main()
{
    fun();
    return 0;
}
```

上述代码结束之后不会delete

我们将其中任意一个类改动成为weak_ptr后，就可以正确输出。



##### c. unique_ptr

该类指针是取代C++98中auto_ptr的产物。unique_ptr 是一个独享所有权的智能指针，它提供了严格意义上的所有权。

```c++

int main() {
    unique_ptr<Test> ptest(new Test("123"));
    unique_ptr<Test> ptest2(new Test("456"));
    ptest->print();//输出123
    // ptest2 = ptest; 这种方法不可行
    ptest2 = std::move(ptest);  //调用了move后ptest2原本的对象会被释放，ptest2对象指向原本ptest对象的内存，输出Test delete 456
    if (ptest == NULL)
        cout << "ptest = NULL\n";   //因为两个unique_ptr不能指向同一内存地址，所以经过前面move后ptest会被赋值NULL，输出ptest=NULL
    Test *p = ptest2.release();     //release成员函数把ptest2指针赋为空，但是并没有释放指针指向的内存，所以此时p指针指向原本ptest2指向的内存
    p->print();//输出123
    ptest.reset(p);//重新绑定对象，原来的对象会被释放掉，但是ptest对象本来就释放过了，所以这里就不会再调用析构函数了
    ptest->print();//输出123
    ptest2 = fun(); //这里可以用=，因为使用了移动构造函数，函数返回一个unique_ptr会自动调用移动构造函数
    ptest2->print();//输出789
    return 0;   //此时程序中还有两个对象，调用两次析构函数释放对象
}
```



