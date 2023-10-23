#include <iostream>
#include <vector>
#include <random>
#include <algorithm>

using namespace std;

// LENGTH和WIDTH若大于10，会导致输出的无线传感网络不美观
constexpr size_t LENGTH = 7;
constexpr size_t WIDTH = 7;
constexpr int CONST_TTL = 15;

struct Event {
    int event_id;                   // 事件名称
    int jumps_to_event;             // 到事件区域的跳数
    int next_neighbors_to_event;    // 到事件区域的下一跳邻居

    Event() = default;
    Event(int curr_event_id, int curr_jumps_to_event, int curr_next_neighbors_to_event) :
        event_id(curr_event_id), jumps_to_event(curr_jumps_to_event),
        next_neighbors_to_event(curr_next_neighbors_to_event) {}
};

struct AgentMessage : public Event {       // 代理信息包含生命期TTL和事件信息
    int TTL;                               // 生命周期

    AgentMessage() = default;
    AgentMessage(int curr_event_id, int curr_jumps_to_event,
                 int curr_next_neighbors_to_event, int curr_TTL) :
        Event(curr_event_id, curr_jumps_to_event, curr_next_neighbors_to_event),
        TTL(curr_TTL) {}
};

struct Node {
    int id;                                 // 传感器节点的编号
    pair<int, int> position;                // 传感器节点的位置
    vector<int> neighbors;                  // 每个传感器节点需要维护邻居节点编号列表
    vector<Event> events_table;             // 每个传感器节点需要维护事件列表
    /*map<int, pair<int, int>> agent_table;*/

    Node(int id, int x, int y) : id(id), position(x, y) {}

    void check_event_table(const Event *event) {  // 多态，直接将AgentMessage转化为Event
        bool exist = false;
        for (auto iter = events_table.begin(); iter != events_table.end();
             ++iter) {
            if (iter->event_id == event->event_id) {
                exist = true;
                if (iter->jumps_to_event > event->jumps_to_event) {
                    events_table.erase(iter);
                    events_table.push_back(*event);
                }
            }
        }
        if (!exist) {
            events_table.push_back(*event);
        }
    }

    bool check_event_table(int event_name) {
        return any_of(events_table.begin(), events_table.end(),
                      [event_name](const Event &event) {
                          return event.event_id == event_name;
                      });
    }

    void print_event_table() {
        cout << "-----------节点" << id << "的事件表为-----------" << endl;
        for (const auto event : events_table) {
            cout << "事件名称为" << event.event_id << "，"
                << "跳数为" << event.jumps_to_event << "，"
                << "下一跳邻居为" << event.next_neighbors_to_event << endl;
        }
        cout << "--------------------------------------" << endl;
    }
};

class Network {
public:

    // 创建无线传感器网络
    void create_network(vector<Node *> &nodes, const size_t length, const size_t width) {
        for (size_t i = 0; i < length * width; ++i) {
            nodes[i] = new Node(static_cast<int>(i), static_cast<int>(i / width),
                                static_cast<int>(i % width));
            get_neighbors(nodes, length, width, i);

            //cout << "node " << nodes[i]->id << " neighbors: ";
            //for (auto neighbor : nodes[i]->neighbors) {
            //    cout << neighbor << " ";
            //}
            //cout << "position is: {" << nodes[i]->position.first << ", "
            //    << nodes[i]->position.second << "}";
            //cout << endl;

        }
    }

    // 打印无线传感网络
    void show_network(const vector<Node *> &nodes, const size_t length, const size_t width) {
        for (size_t i = 0; i < length * width; ++i) {
            if (i != 0 && i % width == 0) {
                cout << endl;
            }
            cout << nodes[i]->id;
            if (i % width != width - 1) {
                if (nodes[i]->id < 10)
                    cout << "---";
                else
                    cout << "--";
            }
            if (i % width == width - 1) {
                cout << endl;
                if (i != length * width - 1) {
                    for (size_t j = 0; j < width; ++j) {
                        cout << "|   ";
                    }
                }
            }
        }
    }

    // 感知到事件的节点概率产生代理信息
    vector<int> generate_agent_message(const vector<Node *> &nodes, const size_t length,
                                       const size_t width) {
        const auto width_i = static_cast<int>(width);
        random_device rd;
        mt19937 gen(rd());
        uniform_int_distribution<int> dis(0, static_cast<int>(nodes.size()) - 1);
        vector<int> event_area;
        const int agent_message_creator = dis(gen);
        event_area.push_back(agent_message_creator - width_i);
        if (agent_message_creator % width_i != 0) {
            event_area.push_back(agent_message_creator - 1);
        }
        event_area.push_back(agent_message_creator);
        if (agent_message_creator % width_i != width_i - 1) {
            event_area.push_back(agent_message_creator + 1);
        }
        event_area.push_back(agent_message_creator + width_i);
        for (auto iter = event_area.begin();
             iter != event_area.end(); /* Empty */) {

            if (*iter < 0 || *iter >= static_cast<int>(length * width)) {
                iter = event_area.erase(iter);
            } else {
                ++iter;
            }
        }
        return event_area;
    }

    // 传播代理信息
    void propagate_agent_message(const vector<Node *> &nodes, int generate_node, int event_name) {
        int curr_TTL = CONST_TTL;
        int curr_jumps_to_event = 0;
        int curr_node = generate_node;
        while (curr_TTL != -1) {
            random_device rd;
            mt19937 gen(rd());
            uniform_int_distribution<int> dis(0,
                                              static_cast<int>(nodes[curr_node]->neighbors.size()) - 1);
            int selected_neighbors = dis(gen);
            int next_node = nodes[curr_node]->neighbors[selected_neighbors];
            next_node = curr_TTL == 0 ? -1 : next_node;
            auto agent_message = new AgentMessage(event_name, curr_jumps_to_event,
                                                  next_node, curr_TTL);
            // 检查事件表中是否已经存在该事件
            nodes[curr_node]->check_event_table(agent_message);
            //nodes[curr_node]->events_table[event_name].push_back(*agent_message);

            cout << "节点" << curr_node << "转发了代理信息，"
                << "事件名称为" << agent_message->event_id << "，"
                << "跳数为" << agent_message->jumps_to_event << "，"
                << "下一跳邻居为" << agent_message->next_neighbors_to_event << "，"
                << "生命期为" << agent_message->TTL << endl;
            nodes[curr_node]->print_event_table();
            cout << endl;
            --curr_TTL;
            ++curr_jumps_to_event;
            curr_node = next_node;
            delete agent_message;
            agent_message = nullptr;
        }
    }

    // 传播搜索信息
    void propagate_search_message(const vector<Node *> &nodes, int sink_node, int event_name) {
        int curr_TTL = CONST_TTL;
        int curr_jumps_to_event = 0;
        int curr_node = sink_node;
        bool res = false;
        vector<int> search_message_path;
        while (curr_TTL != -1) {
            random_device rd;
            mt19937 gen(rd());
            uniform_int_distribution<int> dis(0,
                                              static_cast<int>(nodes[curr_node]->neighbors.size()) - 1);
            int selected_neighbors = dis(gen);
            int next_node = nodes[curr_node]->neighbors[selected_neighbors];
            next_node = curr_TTL == 0 ? -1 : next_node;
            // 检查事件表中是否已经存在该事件
            res = nodes[curr_node]->check_event_table(event_name);
            /*cout << "---------------------------------------------" << res << endl;*/
            cout << "节点" << curr_node << "转发了针对事件" << event_name << "的查找信息，"
                << "跳数为" << curr_jumps_to_event << "，"
                << "下一跳邻居为" << next_node << "，"
                << "生命期为" << curr_TTL << endl;
            nodes[curr_node]->print_event_table();
            search_message_path.push_back(curr_node);
            if (res) {
                cout << "找到了代理消息路径和查询消息路径的交汇处，交汇处为" << curr_node << "号节点" << endl;
                // 沿查询消息的反向路径将代理消息转发给sink节点
                reverse(search_message_path.begin(), search_message_path.end());
                cout << "此时，沿查询消息的反向路径将代理消息转发给sink节点" << endl;
                cout << "路径为：";
                for (size_t i = 0; i < search_message_path.size(); ++i) {
                    cout << search_message_path[i];
                    if (i != search_message_path.size() - 1) {
                        cout << " -> ";
                    }
                }
                break;
            }
            cout << endl;
            --curr_TTL;
            ++curr_jumps_to_event;
            curr_node = next_node;
        }
        if (!res) {
            cout << "没有找到代理消息路径和查询消息路径的交汇处" << endl;
        }
    }

private:
    void get_neighbors(const vector<Node *> &nodes, const size_t length,
                       const size_t width, size_t i) {

        nodes[i]->neighbors.push_back(static_cast<int>(i - width));
        // 如果 nodes[i] 不是宽度的倍数
        if (i % width != 0) {
            nodes[i]->neighbors.push_back(static_cast<int>(i - 1));
        }
        if (i % width != width - 1) {
            nodes[i]->neighbors.push_back(static_cast<int>(i + 1));
        }
        nodes[i]->neighbors.push_back(static_cast<int>(i + width));

        for (auto iter = nodes[i]->neighbors.begin();
             iter != nodes[i]->neighbors.end(); /* Empty */) {

            if (*iter < 0 || *iter >= static_cast<int>(length * width)) {
                iter = nodes[i]->neighbors.erase(iter);
            } else {
                ++iter;
            }
        }
    }

};

// 输出生成代理信息的节点
int show_generate_node(const size_t C_LENGTH, const size_t C_WIDTH,
                       const vector<Node *> &nodes, Network *network) {

    cout << "现在，假设这些无线传感网络中序号为";
    const auto res = network->generate_agent_message(nodes, C_LENGTH, C_WIDTH);
    for (int i = 0; i < static_cast<int>(res.size()); ++i) {
        cout << res[i];
        if (i != static_cast<int>(res.size()) - 1)
            cout << ", ";
        else
            cout << "的节点感知到了事件。" << endl;
    }
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<int> dis(0, static_cast<int>(res.size()) - 1);
    const int generate_node = dis(gen);
    cout << "此时，假设节点" << res[generate_node] << "产生了代理信息。" << endl;
    return res[generate_node];
}

int generate_sink(const vector<Node *> &nodes) {
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<int> dis(0, static_cast<int>(nodes.size()) - 1);
    const int sink = dis(gen);
    cout << "sink节点为：" << nodes[sink]->id << endl;
    return sink;
}

/**
 * 传感器网络的长：length，宽：width
 * 例：
 *
 *      0---1---2---3
 *      |   |   |   |
 *      4---5---6---7
 *      |   |   |   |
 *      8---9---10--11
 *
 *  这是一个 length = 3, width = 4 的传感器网络
 */
int main() {
    vector<Node *> nodes(WIDTH * LENGTH);
    auto network = new Network();
    network->create_network(nodes, LENGTH, WIDTH);
    cout << "当前的无线传感网络为：" << endl;
    network->show_network(nodes, LENGTH, WIDTH);
    cout << endl;
    // 生成代理信息的节点id
    const int generate_node = show_generate_node(LENGTH, WIDTH, nodes, network);
    // 传播代理信息 
    int event_name = 0;
    network->propagate_agent_message(nodes, generate_node, event_name);
    // 生成sink节点
    const int sink_node = generate_sink(nodes);
    network->propagate_search_message(nodes, sink_node, event_name);

    // 释放内存
    delete network;
    network = nullptr;
    for (auto node : nodes) {
        delete node;
        node = nullptr;
    }
    nodes.clear();
    return 0;
}
