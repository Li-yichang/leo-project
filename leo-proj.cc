#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/mobility-module.h"

#include <fstream>
#include <vector>
#include <sstream>
#include <iostream>
#include <map>
#include <set>

using namespace ns3;
using std::vector;
using std::string;
using std::map;
using std::set;

NS_LOG_COMPONENT_DEFINE("LeoPathForward");

static std::ofstream g_output;
static double g_startTime = 0.0;
static double g_leo1SendTime = 0.0;
static double g_endTime = 0.0;
static std::vector<int> g_path; // path user input: values in 1..7

// IpMap[from][to] = Ipv4Address of "to" as seen from "from" (i.e. the address we send to)
using IpMap = std::map<int, std::map<int, Ipv4Address>>;

// Calculate propagation delay based on positions (speed of light)
static double CalcPropDelay(Ptr<Node> a, Ptr<Node> b)
{
    Vector pa = a->GetObject<MobilityModel>()->GetPosition();
    Vector pb = b->GetObject<MobilityModel>()->GetPosition();
    double dist = (pb - pa).GetLength();
    return dist / 3e8;
}

// Parse user input path e.g. "1 2 3 5"
static std::vector<int> ParsePathLine(const std::string &line)
{
    std::vector<int> out;
    std::stringstream ss(line);
    int x;
    while (ss >> x)
    {
        if (x >= 1 && x <= 7) out.push_back(x);
    }
    return out;
}

class SourceApp : public Application
{
public:
    void Setup(Ptr<Socket> sock, Ipv4Address dest, uint16_t port, uint32_t pktSize)
    {
        m_socket = sock;
        m_dest = dest;
        m_port = port;
        m_pktSize = pktSize;
    }

private:
    virtual void StartApplication() override
    {
        Simulator::Schedule(Seconds(0.1), &SourceApp::DoSend, this);
    }

    void DoSend()
    {
        Ptr<Packet> pkt = Create<Packet>(m_pktSize);
        m_socket->SendTo(pkt, 0, InetSocketAddress(m_dest, m_port));
        // record start time at actual send
        if (g_startTime == 0.0) g_startTime = Simulator::Now().GetSeconds();
    }

    Ptr<Socket> m_socket;
    Ipv4Address m_dest;
    uint16_t m_port;
    uint32_t m_pktSize;
};

class DynamicForwardApp : public Application
{
public:
    void Setup(Ptr<Socket> recvSock,
               const std::vector<Ptr<Node>>& nodes,
               const IpMap &ipMap,
               int nodeIndex,
               const std::vector<int>& path,
               int destNodeId,
               bool compress,
               double ratio,
               uint16_t port)
    {
        m_recvSock = recvSock;
        m_nodes = nodes;
        m_ipMap = ipMap;
        m_nodeIndex = nodeIndex;
        m_path = path;
        m_destNodeId = destNodeId;
        m_compress = compress;
        m_ratio = ratio;
        m_port = port;
        m_totalCompressedBits = 0.0;
    }

    double GetTotalCompressedBits() const { return m_totalCompressedBits; }

private:
    virtual void StartApplication() override
    {
        if (m_recvSock)
            m_recvSock->SetRecvCallback(MakeCallback(&DynamicForwardApp::HandleRecv, this));
    }

    // find next node id according to m_path
    int GetNextHopNodeId()
    {
        for (size_t i = 0; i < m_path.size(); ++i)
        {
            if (m_path[i] == m_nodeIndex)
            {
                if (i + 1 < m_path.size()) return m_path[i + 1];
                else return m_destNodeId;
            }
        }
        return -1;  
    }

    void HandleRecv(Ptr<Socket> sock)
    {
        Ptr<Packet> pkt;
        while ((pkt = sock->Recv()))
        {
            if (m_nodeIndex == m_destNodeId)
            {
                if (g_endTime == 0.0) g_endTime = Simulator::Now().GetSeconds();
                return;
            }

            uint32_t forwardSize = pkt->GetSize();

            // compress if this node compresses
            if (m_compress)
            {
                forwardSize = static_cast<uint32_t>(std::max<uint32_t>(1, static_cast<uint32_t>(forwardSize * m_ratio)));
                pkt = Create<Packet>(forwardSize);
                m_totalCompressedBits += (double)forwardSize * 8.0;

                if (g_leo1SendTime == 0.0)
                    g_leo1SendTime = Simulator::Now().GetSeconds();
            }


            int nextNode = GetNextHopNodeId();
            if (nextNode < 0) {
                return;
            }
            auto itFrom = m_ipMap.find(m_nodeIndex);
            if (itFrom == m_ipMap.end()) return;
            auto itTo = itFrom->second.find(nextNode);
            if (itTo == itFrom->second.end()) return;

            Ipv4Address nextIp = itTo->second;
            if (nextIp == Ipv4Address::GetZero()) return;

            Ptr<Socket> sendSock = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
            sendSock->SendTo(pkt, 0, InetSocketAddress(nextIp, m_port));
        }
    }

private:
    Ptr<Socket> m_recvSock;
    std::vector<Ptr<Node>> m_nodes;
    IpMap m_ipMap;
    int m_nodeIndex;
    std::vector<int> m_path;
    int m_destNodeId;
    bool m_compress;
    double m_ratio;
    uint16_t m_port;
    double m_totalCompressedBits;
};

static void GroundRecvCb(Ptr<Socket> sock)
{
    Ptr<Packet> pkt;
    while ((pkt = sock->Recv()))
    {
        if (g_endTime == 0.0) g_endTime = Simulator::Now().GetSeconds();
    }
}

static void CreateP2PAndAssign(NodeContainer &nodes, int a, int b,
                               PointToPointHelper &p2p,
                               Ipv4AddressHelper &ipv4helper,
                               IpMap &ipMap,
                               uint32_t &netCounter)
{
    double dsec = CalcPropDelay(nodes.Get(a), nodes.Get(b));
    p2p.SetChannelAttribute("Delay", TimeValue(Seconds(dsec)));

    NetDeviceContainer ndc = p2p.Install(nodes.Get(a), nodes.Get(b));

    std::ostringstream base;
    base << "10." << (netCounter / 256) << "." << (netCounter % 256) << ".0";
    ipv4helper.SetBase(base.str().c_str(), "255.255.255.0");
    Ipv4InterfaceContainer ifc = ipv4helper.Assign(ndc);

    ipMap[a][b] = ifc.GetAddress(1);
    ipMap[b][a] = ifc.GetAddress(0);

    ++netCounter;
}

void RunExperiment(const std::vector<int> &path, double ratio, uint32_t pktSize)
{
    g_startTime = 0.0;
    g_leo1SendTime = 0.0;
    g_endTime = 0.0;

    const int N = 9; 
    NodeContainer nodes;
    nodes.Create(N);

    // positions
    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    posAlloc->Add(Vector(0, 0, 0));             // Source 0
    posAlloc->Add(Vector(80, 20, 600e3));  // 1
    posAlloc->Add(Vector(982, 340, 650e3));  // 2
    posAlloc->Add(Vector(1020, 3490, 700e3)); // 3
    posAlloc->Add(Vector(2320, 20000, 750e3)); // 4
    posAlloc->Add(Vector(673450, 9430e3, 800e3));// 5
    posAlloc->Add(Vector(4657, 94200, 850e3)); // 6
    posAlloc->Add(Vector(13, 340e3, 600e3));   // 7
    posAlloc->Add(Vector(0, 270e3, 0));         // Ground 8

    MobilityHelper mobility;
    mobility.SetPositionAllocator(posAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    InternetStackHelper internet;
    internet.Install(nodes);

    // P2P helper
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("100Kbps"));

    IpMap ipMap;
    Ipv4AddressHelper ipv4helper;
    uint32_t netCounter = 1; // unique counter for network bases

    if (!path.empty())
    {
        int firstNode = path.front();
        CreateP2PAndAssign(nodes, 0, firstNode, p2p, ipv4helper, ipMap, netCounter);

        // adjacent pairs in path
        for (size_t i = 0; i + 1 < path.size(); ++i)
        {
            int a = path[i];
            int b = path[i + 1];
            if (ipMap[a].find(b) == ipMap[a].end())
            {
                CreateP2PAndAssign(nodes, a, b, p2p, ipv4helper, ipMap, netCounter);
            }
        }

        // last -> ground
        int last = path.back();
        CreateP2PAndAssign(nodes, last, 8, p2p, ipv4helper, ipMap, netCounter);
    }
    else
    {
        // no path input -> default chain 0-1-2-...-8
        CreateP2PAndAssign(nodes, 0, 1, p2p, ipv4helper, ipMap, netCounter);
        CreateP2PAndAssign(nodes, 1, 8, p2p, ipv4helper, ipMap, netCounter);
    }

    const uint16_t PORT = 8080;

    Ptr<Socket> srcSock = Socket::CreateSocket(nodes.Get(0), UdpSocketFactory::GetTypeId());
    Ptr<SourceApp> srcApp = CreateObject<SourceApp>();

    Ipv4Address firstDestIp = Ipv4Address::GetZero();
    if (!path.empty())
    {
        int firstNode = path.front();
        if (ipMap[0].find(firstNode) != ipMap[0].end())
            firstDestIp = ipMap[0][firstNode];
    }
    if (firstDestIp == Ipv4Address::GetZero())
    {
        if (ipMap[0].find(1) != ipMap[0].end()) firstDestIp = ipMap[0][1];
    }
    if (firstDestIp == Ipv4Address::GetZero())
    {
        std::cout << "No valid firstDestIp for source. Skipping experiment." << std::endl;
        g_output << ratio << "," << pktSize << "," << 0 << "," << 0 << "," << 0 << std::endl;
        return;
    }

    srcApp->Setup(srcSock, firstDestIp, PORT, pktSize);
    nodes.Get(0)->AddApplication(srcApp);
    srcApp->SetStartTime(Seconds(0.5));
    srcApp->SetStopTime(Seconds(2.0));

    // Install DynamicForwardApp on LEOS
    std::vector<Ptr<DynamicForwardApp>> leoApps(N);
    for (int i = 1; i <= 7; ++i)
    {
        Ptr<Socket> recvSock = Socket::CreateSocket(nodes.Get(i), UdpSocketFactory::GetTypeId());
        InetSocketAddress anyAddr = InetSocketAddress(Ipv4Address::GetAny(), PORT);
        recvSock->Bind(anyAddr);

        Ptr<DynamicForwardApp> app = CreateObject<DynamicForwardApp>();

        bool compress = (!path.empty() && i == path.front()); 

        app->Setup(recvSock, std::vector<Ptr<Node>>(nodes.Begin(), nodes.End()),
                ipMap, i, path, 8, compress, ratio, PORT);

        nodes.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.5));
        app->SetStopTime(Seconds(40.0));
        leoApps[i] = app;
    }


    // Ground receive
    Ptr<Socket> gSock = Socket::CreateSocket(nodes.Get(8), UdpSocketFactory::GetTypeId());
    InetSocketAddress anyAddr = InetSocketAddress(Ipv4Address::GetAny(), PORT);
    gSock->Bind(anyAddr);
    gSock->SetRecvCallback(MakeCallback(&GroundRecvCb));

    Simulator::Stop(Seconds(60.0));
    Simulator::Run();
    Simulator::Destroy();

    // Compute throughputs & total time
    double upBits = (double)pktSize * 8.0;
    const double eps = 1e-9;

    double upTime = (g_leo1SendTime > 0.0 && g_startTime > 0.0) ? (g_leo1SendTime - g_startTime) : eps;
    double upThroughput = upBits / upTime / 1e6;

    double downBits = 0.0;
    for (int idx : g_path) {
        if (idx >= 1 && idx <= 7 && leoApps[idx] && leoApps[idx]->GetTotalCompressedBits() > 0) {
            downBits = leoApps[idx]->GetTotalCompressedBits();
            break;
        }
    }
    if (downBits <= 0.0) downBits = (double)pktSize * 8.0 * ratio;

    double downTime = (g_endTime > g_leo1SendTime && g_leo1SendTime > 0.0) ? (g_endTime - g_leo1SendTime) : 1e-9;
    double downThroughput = downBits / downTime / 1e6;


    double totalTime = (g_endTime > g_startTime && g_startTime > 0.0) ? (g_endTime - g_startTime) : 0.0;

    g_output << ratio << "," << pktSize << "," << upThroughput << "," << downThroughput << "," << totalTime << std::endl;
    std::cout << "ratio=" << ratio << " pkt=" << pktSize
              << " Up(Mbps)=" << upThroughput
              << " Down(Mbps)=" << downThroughput
              << " Total(s)=" << totalTime << std::endl;
}

int main(int argc, char* argv[])
{
    std::cout << "there are 1 to 7 SATs, input path sequence,e.g. 1 2 3 5: " << std::endl;
    std::string line;
    std::getline(std::cin, line);
    if (line.size() == 0) std::getline(std::cin, line);
    g_path = ParsePathLine(line);

    std::cout << "path already set: ";
    for (int v : g_path) std::cout << v << " ";
    std::cout << std::endl;

    g_output.open("leo-results.csv");
    g_output << "Ratio,PacketSize,Up(Mbps),Down(Mbps),TotalTime(s)" << std::endl;

    double ratios[] = {1.0, 0.5, 0.2};
    uint32_t pkts[] = {1000, 5000, 10000};

    for (double r : ratios)
    {
        for (uint32_t p : pkts)
        {
            RunExperiment(g_path, r, p);
        }
    }

    g_output.close();
    std::cout << "All experiments completed." << std::endl;
    return 0;
}
