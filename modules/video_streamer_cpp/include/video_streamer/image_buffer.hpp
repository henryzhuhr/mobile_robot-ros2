
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include "interfaces/msg/lanes.hpp"

typedef struct
{
    std_msgs::msg::Header header;
    cv::Mat img;
    uint8_t flag = 0;
    interfaces::msg::Lanes lanes;
} BufferData;

/**
 * @brief 图像缓冲队列节点
 */
struct BufferNode
{
    BufferData val;
    BufferNode *next;
    BufferNode() { this->next = nullptr; }
    BufferNode(BufferData x) : val(x), next(nullptr) {}
};

/**
 * @brief 图像缓冲队列
 */
class ImageBuffer
{
private:
    BufferNode *_head;
    BufferNode *_tail;
    int _size = 0;
    int _max_size = -1;

public:
    ImageBuffer(const int &max_size = -1) : _max_size(max_size)
    {
        this->_head = new BufferNode();
        this->_tail = this->_head;
    }
    ~ImageBuffer()
    {
        BufferNode *p = this->_head;
        BufferNode *q = p;
        while (p != nullptr)
        {
            q = p->next;
            delete p;
            p = q;
        }
    }

    int size() { return this->_size; }

    void enqueue(BufferData data)
    {
        if (this->_max_size > 0 && this->_size >= this->_max_size)
        {
            this->dequeue();
        }
        BufferNode *newNode = new BufferNode(data);
        this->_tail->next = newNode;
        this->_tail = newNode;
        this->_size++;
    }
    BufferData *dequeue()
    {
        if (this->_size > 0)
        {
            BufferNode *p = this->_head->next;
            this->_head->next = p->next;
            if (p == this->_tail)
            {
                this->_tail = this->_head;
            }
            this->_size--;
            return &(p->val);
        }
        else
        {
            return nullptr;
        }
    }

    uint8_t try_set(BufferData data)
    {
        if (this->_size > 0)
        {
            BufferNode *p = this->_head->next;
            while (p != nullptr)
            {
                if (data.header.stamp == p->val.header.stamp)
                {
                    if ((data.flag & 2) != 0)
                    {
                        p->val.lanes = data.lanes;
                        p->val.flag |= 2;
                        return 2;
                    }
                    else
                    {
                        return 0;
                    }
                }
                p = p->next;
            }
            return 0;
        }
        else
        {
            return 0;
        }
    }
};
