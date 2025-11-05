#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/mobility-module.h"
#include <fstream>
#include <vector>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("LEOISLproj");

static ofstream g_output;
static double g_startTime = 0;
static double g_endTime = 0;
static double g_leoSendTime = 0;

// compress part
class LEOApp : public Application
{
public:
    LEOApp() {}
    virtual ~LEOApp() {}

    void Setup(Ptr<Socket> recvSock, double ratio, Ipv4Address dest)
    {
        m_recvSock = recvSock;
        m_ratio = ratio;
        m_dest = dest;
    }

    double GetTotalCompressedBits() const { return m_totalCompressedBits; }

private:
    virtual void StartApplication() override
    {
        if (m_recvSock)
        {
            m_recvSock->SetRecvCallback(MakeCallback(&LEOApp::ReceivePacket, this));
        }
    }

    void ReceivePacket(Ptr<Socket> socket)
    {
        Ptr<Packet> pkt;
        while ((pkt = socket->Recv()))
        {
            uint32_t newSize = static_cast<uint32_t>(pkt->GetSize() * m_ratio);
            Ptr<Packet> compressed = Create<Packet>(newSize);
            m_totalCompressedBits += newSize * 8;
            g_leoSendTime = Simulator::Now().GetSeconds();

            Ptr<Socket> sendSock = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
            sendSock->SendTo(compressed, 0, InetSocketAddress(m_dest, 8080));
        }
    }

    Ptr<Socket> m_recvSock;
    double m_ratio = 1.0;
    double m_totalCompressedBits = 0;
    Ipv4Address m_dest;
};

// iSL Forwarding A
class ISLForwardApp : public Application
{
public:
    ISLForwardApp() {}
    virtual ~ISLForwardApp() {}

    void Setup(Ptr<Socket> recvSock, Ipv4Address dest)
    {
        m_recvSock = recvSock;
        m_dest = dest;
    }

private:
    virtual void StartApplication() override
    {
        if (m_recvSock)
        {
            m_recvSock->SetRecvCallback(MakeCallback(&ISLForwardApp::ForwardPacket, this));
        }
    }

    void ForwardPacket(Ptr<Socket> socket)
    {
        Ptr<Packet> pkt;
        while ((pkt = socket->Recv()))
        {
            Ptr<Socket> sendSock = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
            sendSock->SendTo(pkt, 0, InetSocketAddress(m_dest, 8080));
        }
    }

    Ptr<Socket> m_recvSock;
    Ipv4Address m_dest;
};

//  Ground 
void GroundReceive(Ptr<Socket> socket)
{
    Ptr<Packet> pkt;
    while ((pkt = socket->Recv()))
    {
        g_endTime = Simulator::Now().GetSeconds();
    }
}

// Propagation Delay
double CalculatePropDelay(Ptr<Node> a, Ptr<Node> b)
{
    Vector posA = a->GetObject<MobilityModel>()->GetPosition();
    Vector posB = b->GetObject<MobilityModel>()->GetPosition();
    double distance = (posB - posA).GetLength();
    return distance / 3e8;
}

// Source 
void SendPacketAtSource(Ptr<Socket> sock, Ipv4Address dest, uint32_t pktSize)
{
    Ptr<Packet> pkt = Create<Packet>(pktSize);
    sock->SendTo(pkt, 0, InetSocketAddress(dest, 8080));
    g_startTime = Simulator::Now().GetSeconds();
}

void RunExperiment(double ratio, uint32_t pktSize)
{
    NodeContainer nodes;
    nodes.Create(9); // 0=Source,1=LEO,2~7=ISL,8=Ground

    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    posAlloc->Add(Vector(0, 0, 0));        // Source
    posAlloc->Add(Vector(34980, 1020, 600e3));    // LEO1
    posAlloc->Add(Vector(9820, 34000, 650e3));     
    posAlloc->Add(Vector(24000, 34900, 700e3));     
    posAlloc->Add(Vector(23200, 20000, 750e3));     
    posAlloc->Add(Vector(673450, 94350, 800e3));    
    posAlloc->Add(Vector(46570, 94200, 850e3));     
    posAlloc->Add(Vector(13434, 340, 900e3));    
    posAlloc->Add(Vector(0, 270e3, 0));        // Ground

    MobilityHelper mobility;
    mobility.SetPositionAllocator(posAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    InternetStackHelper internet;
    internet.Install(nodes);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("100Kbps"));
    vector<NetDeviceContainer> devices;
    for (int i = 0; i < 8; ++i)
    {
        double delay = CalculatePropDelay(nodes.Get(i), nodes.Get(i + 1));
        p2p.SetChannelAttribute("Delay", TimeValue(Seconds(delay)));
        devices.push_back(p2p.Install(nodes.Get(i), nodes.Get(i + 1)));
    }

    Ipv4AddressHelper ipv4;
    vector<Ipv4InterfaceContainer> interfaces;
    for (int i = 0; i < 8; ++i)
    {
        string base = "10.0." + to_string(i) + ".0";
        ipv4.SetBase(base.c_str(), "255.255.255.0");
        interfaces.push_back(ipv4.Assign(devices[i]));
    }

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // Ground IP
    Ipv4Address groundAddr = interfaces.back().GetAddress(1);

    // Source 
    Ptr<Socket> sourceSock = Socket::CreateSocket(nodes.Get(0), UdpSocketFactory::GetTypeId());
    Simulator::Schedule(Seconds(1.0), &SendPacketAtSource, sourceSock, interfaces[0].GetAddress(1), pktSize);

    // LEO 
    Ptr<Socket> leoSock = Socket::CreateSocket(nodes.Get(1), UdpSocketFactory::GetTypeId());
    leoSock->Bind(InetSocketAddress(Ipv4Address::GetAny(), 8080));
    Ptr<LEOApp> leoApp = CreateObject<LEOApp>();
    leoApp->Setup(leoSock, ratio, groundAddr);
    nodes.Get(1)->AddApplication(leoApp);
    leoApp->SetStartTime(Seconds(0.5));
    leoApp->SetStopTime(Seconds(40.0));

    // ISL 
    for (int i = 2; i <= 7; ++i)
    {
        Ptr<Socket> islSock = Socket::CreateSocket(nodes.Get(i), UdpSocketFactory::GetTypeId());
        islSock->Bind(InetSocketAddress(Ipv4Address::GetAny(), 8080));
        Ptr<ISLForwardApp> islApp = CreateObject<ISLForwardApp>();
        islApp->Setup(islSock, groundAddr);
        nodes.Get(i)->AddApplication(islApp);
        islApp->SetStartTime(Seconds(0.5));
        islApp->SetStopTime(Seconds(40.0));
    }

    // Ground 
    Ptr<Socket> groundSock = Socket::CreateSocket(nodes.Get(8), UdpSocketFactory::GetTypeId());
    groundSock->Bind(InetSocketAddress(Ipv4Address::GetAny(), 8080));
    groundSock->SetRecvCallback(MakeCallback(&GroundReceive));

    Simulator::Stop(Seconds(50.0));
    Simulator::Run();
    Simulator::Destroy();

    double upBits = pktSize * 8;
    double upTime = g_leoSendTime - g_startTime;
    double upThroughput = upBits / upTime / 1e6;

    double downBits = leoApp->GetTotalCompressedBits();
    double downTime = g_endTime - g_leoSendTime;
    double downThroughput = downBits / downTime / 1e6;

    double totalTime = g_endTime - g_startTime;

    g_output << ratio << "," << pktSize << "," << upThroughput << "," << downThroughput << "," << totalTime << endl;

    cout << "Ratio=" << ratio
         << " Pkt=" << pktSize
         << " Up=" << upThroughput
         << " Down=" << downThroughput
         << " Total=" << totalTime << endl;
}

int main(int argc, char* argv[])
{
    g_output.open("leo-results.csv");
    g_output << "CompressionRatio,PacketSize(byte),UpThroughput(Mbps),DownThroughput(Mbps),TotalTime(s)" << endl;

    double ratios[] = {1.0, 0.5, 0.2};
    uint32_t pkts[] = {1000, 5000, 10000};

    for (auto r : ratios)
        for (auto p : pkts)
            RunExperiment(r, p);

    g_output.close();
    cout << "All experiments completed." << endl;
    return 0;
}

