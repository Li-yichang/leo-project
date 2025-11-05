#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/mobility-module.h"
#include <fstream>
#include <vector>
#include <sstream>

using namespace ns3;
using std::vector;
using std::string;

NS_LOG_COMPONENT_DEFINE("LeoProj");

static std::ofstream g_output;
static double g_startTime = 0.0;
static double g_leo1SendTime = 0.0;
static double g_endTime = 0.0;

static double CalcPropDelay(Ptr<Node> a, Ptr<Node> b)
{
    Vector pa = a->GetObject<MobilityModel>()->GetPosition();
    Vector pb = b->GetObject<MobilityModel>()->GetPosition();
    double dist = (pb - pa).GetLength();
    return dist / 3e8;
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
        g_startTime = Simulator::Now().GetSeconds();
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
               const std::vector<Ipv4InterfaceContainer>& ifcs,
               int nodeIndex,
               int destNodeId,
               bool compress,
               double ratio,
               uint16_t port)
    {
        m_recvSock = recvSock;
        m_nodes = nodes;
        m_ifcs = ifcs;
        m_nodeIndex = nodeIndex;
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

    // choose next hop
    int GetNextHop() { return m_nodeIndex + 1; }

    Ipv4Address GetIpOfNodeOnLink(int current, int next)
    {
        int linkIdx = std::min(current, next);
        if (linkIdx < 0 || linkIdx >= (int)m_ifcs.size()) return Ipv4Address::GetZero();
        if (next == linkIdx) return m_ifcs[linkIdx].GetAddress(0);
        else return m_ifcs[linkIdx].GetAddress(1);
    }

    void HandleRecv(Ptr<Socket> sock)
    {
        Ptr<Packet> pkt;
        while ((pkt = sock->Recv()))
        {
            if (m_nodeIndex == m_destNodeId)
            {
                g_endTime = Simulator::Now().GetSeconds();
                return;
            }

            uint32_t forwardSize = pkt->GetSize();

            if (m_compress)
            {
                forwardSize = static_cast<uint32_t>(forwardSize * m_ratio);
                pkt = Create<Packet>(forwardSize);
                m_totalCompressedBits += (double)forwardSize * 8.0;
                if (g_leo1SendTime == 0.0) g_leo1SendTime = Simulator::Now().GetSeconds();
            }

            int next = GetNextHop();
            if (next > m_destNodeId) return;

            Ipv4Address nextIp = GetIpOfNodeOnLink(m_nodeIndex, next);
            if (nextIp == Ipv4Address::GetZero()) return;

            Ptr<Socket> sendSock = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
            sendSock->SendTo(pkt, 0, InetSocketAddress(nextIp, m_port));
        }
    }

private:
    Ptr<Socket> m_recvSock;
    std::vector<Ptr<Node>> m_nodes;
    std::vector<Ipv4InterfaceContainer> m_ifcs;
    int m_nodeIndex;
    int m_destNodeId;
    bool m_compress;
    double m_ratio;
    uint16_t m_port;
    double m_totalCompressedBits;
};

// Ground receive 
static void GroundRecvCb(Ptr<Socket> sock)
{
    Ptr<Packet> pkt;
    while ((pkt = sock->Recv()))
    {
        g_endTime = Simulator::Now().GetSeconds();
    }
}

void RunExperiment(double ratio, uint32_t pktSize)
{
    const int N = 9; // 0=Source,1=LEO1,..7=LEO7,8=Ground
    NodeContainer nodes;
    nodes.Create(N);

    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    posAlloc->Add(Vector(0, 0, 0));             // Source
    posAlloc->Add(Vector(34980, 1020, 600e3));  // LEO1
    posAlloc->Add(Vector(9820, 34000, 650e3));  
    posAlloc->Add(Vector(24000, 34900, 700e3)); 
    posAlloc->Add(Vector(23200, 20000, 750e3)); 
    posAlloc->Add(Vector(673450, 94350, 800e3));  
    posAlloc->Add(Vector(46570, 94200, 850e3));  
    posAlloc->Add(Vector(13434, 340, 900e3));    
    posAlloc->Add(Vector(0, 270e3, 0));         // Ground

    MobilityHelper mobility;
    mobility.SetPositionAllocator(posAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    InternetStackHelper internet;
    internet.Install(nodes);

    //  P2P 
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("100Kbps"));
    std::vector<NetDeviceContainer> devs;
    for (int i = 0; i < N - 1; ++i)
    {
        double dsec = CalcPropDelay(nodes.Get(i), nodes.Get(i + 1));
        p2p.SetChannelAttribute("Delay", TimeValue(Seconds(dsec)));
        devs.push_back(p2p.Install(nodes.Get(i), nodes.Get(i + 1)));
    }

    // IP assignment
    Ipv4AddressHelper ipv4;
    std::vector<Ipv4InterfaceContainer> ifcs;
    for (int i = 0; i < (int)devs.size(); ++i)
    {
        std::ostringstream base;
        base << "10.0." << i << ".0";
        ipv4.SetBase(base.str().c_str(), "255.255.255.0");
        ifcs.push_back(ipv4.Assign(devs[i]));
    }

    const uint16_t PORT = 8080;

    // Source -> LEO1
    Ptr<Socket> srcSock = Socket::CreateSocket(nodes.Get(0), UdpSocketFactory::GetTypeId());
    Ptr<SourceApp> srcApp = CreateObject<SourceApp>();
    Ipv4Address leo1Ip = ifcs[0].GetAddress(1);
    srcApp->Setup(srcSock, leo1Ip, PORT, pktSize);
    nodes.Get(0)->AddApplication(srcApp);
    srcApp->SetStartTime(Seconds(0.5));
    srcApp->SetStopTime(Seconds(2.0));

    // LEO1~LEO7 applications
    std::vector<Ptr<Socket>> recvSocks(N);
    Ptr<DynamicForwardApp> node1AppPtr = 0;
    for (int i = 1; i <= 7; ++i)
    {
        recvSocks[i] = Socket::CreateSocket(nodes.Get(i), UdpSocketFactory::GetTypeId());
        InetSocketAddress anyAddr = InetSocketAddress(Ipv4Address::GetAny(), PORT);
        recvSocks[i]->Bind(anyAddr);

        Ptr<DynamicForwardApp> app = CreateObject<DynamicForwardApp>();
        bool compress = (i == 1); // LEO1 compress
        std::vector<Ptr<Node>> nodePtrs;
        for (int j = 0; j < N; ++j) nodePtrs.push_back(nodes.Get(j));
        app->Setup(recvSocks[i], nodePtrs, ifcs, i, N - 1, compress, ratio, PORT);
        nodes.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.5));
        app->SetStopTime(Seconds(40.0));
        if (i == 1) node1AppPtr = app;
    }

    // Ground receive
    Ptr<Socket> gSock = Socket::CreateSocket(nodes.Get(N - 1), UdpSocketFactory::GetTypeId());
    InetSocketAddress anyAddr = InetSocketAddress(Ipv4Address::GetAny(), PORT);
    gSock->Bind(anyAddr);
    gSock->SetRecvCallback(MakeCallback(&GroundRecvCb));

    Simulator::Stop(Seconds(60.0));
    Simulator::Run();
    Simulator::Destroy();

    double upBits = (double)pktSize * 8.0;
    double upTime = (g_leo1SendTime > 0.0) ? (g_leo1SendTime - g_startTime) : 1e-9;
    double upThroughput = upBits / upTime / 1e6;

    double downBits = node1AppPtr ? node1AppPtr->GetTotalCompressedBits() : (double)pktSize * 8.0 * ratio;
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
    g_output.open("leo-results.csv");
    g_output << "Ratio,PacketSize,Up(Mbps),Down(Mbps),TotalTime(s)" << std::endl;

    double ratios[] = {1.0, 0.5, 0.2};
    uint32_t pkts[] = {1000, 5000, 10000};

    for (double r : ratios)
        for (uint32_t p : pkts)
            RunExperiment(r, p);

    g_output.close();
    std::cout << "All experiments completed." << std::endl;
    return 0;
}
