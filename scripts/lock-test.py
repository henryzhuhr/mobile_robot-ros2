from time import sleep


class Test:
    class Lock:
        NONE = 0   # 没有线程控制
        JOY = 1    # 手柄控制
        AUTO = 2   # 自动控制

    __state = Lock.NONE

    def __init__(self) -> None:
        pass

    def change(self):
        self.state = Test.Lock.AUTO
    
    @property
    def state(self):
        return Test.__state

    @state.setter
    def state(self, num):
        Test.__state = num

test1 = Test()
test2 = Test()


print(test1.state,test2.state)
test1.state = Test.Lock.JOY
print(test1.state,test2.state)
test1.change()
print(test1.state,test2.state)
test2.state=Test.Lock.JOY
print(test1.state,test2.state)