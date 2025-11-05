#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/mobility-module.h"
#include <fstream>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("LEOproj");

static ofstream g_output;
static double g_startTime = 0;
static double g_endTime = 0;
static double g_leoSendTime = 0;

class LEOApp : public Application
{
public:
    LEOApp() {}
    virtual ~LEOApp() {}

    void Setup(Ptr<Socket> recvSock, double ratio)
    {
        m_recvSock = recvSock;
        m_ratio = ratio;
    }

    double GetTotalCompressedBits() const { return m_totalCompressedBits; }

private:
    virtual void StartApplication(void)
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

            g_leoSendTime = Simulator::Now().GetSeconds();

            Ptr<Socket> sendSock = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
            Ipv4Address groundAddr("10.0.7.2");
            sendSock->Connect(InetSocketAddress(groundAddr, 8080));
            sendSock->Send(compressed);

            m_totalCompressedBits += newSize * 8;
        }
    }

    Ptr<Socket> m_recvSock;
    double m_ratio = 1.0;
    double m_totalCompressedBits = 0;
};

void GroundReceive(Ptr<Socket> socket)
{
    Ptr<Packet> pkt;
    while ((pkt = socket->Recv()))
    {
        g_endTime = Simulator::Now().GetSeconds();
    }
}

double CalculatePropDelay(Ptr<Node> a, Ptr<Node> b)
{
    Vector posA = a->GetObject<MobilityModel>()->GetPosition();
    Vector posB = b->GetObject<MobilityModel>()->GetPosition();
    double distance = (posB - posA).GetLength();
    return distance / 3e8; // light
}

void SendPacketAtSource(Ptr<Socket> sock, Address addr, uint32_t pktSize)
{
    Ptr<Packet> pkt = Create<Packet>(pktSize);
    sock->SendTo(pkt, 0, addr);
    g_startTime = Simulator::Now().GetSeconds();
}

void RunExperiment(double ratio, uint32_t pktSize)
{
    NodeContainer nodes;
    nodes.Create(9); // 0=Source, 1=LEO, 2-7=ISL, 8=Ground

    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    posAlloc->Add(Vector(0, 0, 0));        // Source
    posAlloc->Add(Vector(4350, 18470, 600e3));    // LEO1
    posAlloc->Add(Vector(346230, 5410, 650e3));    
    posAlloc->Add(Vector(230, 13450, 700e3));    
    posAlloc->Add(Vector(2340, 5260, 750e3));    
    posAlloc->Add(Vector(78340, 310, 800e3));    
    posAlloc->Add(Vector(350, 2640, 850e3));    
    posAlloc->Add(Vector(2400, 36000, 900e3));    
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

    // Source 
    Ptr<Socket> sourceSock = Socket::CreateSocket(nodes.Get(0), UdpSocketFactory::GetTypeId());
    Address leoAddr = InetSocketAddress(interfaces[0].GetAddress(1), 8080);

    Simulator::Schedule(Seconds(1.0), &SendPacketAtSource, sourceSock, leoAddr, pktSize);

    // LEO 
    Ptr<Socket> leoRecvSock = Socket::CreateSocket(nodes.Get(1), UdpSocketFactory::GetTypeId());
    leoRecvSock->Bind(InetSocketAddress(Ipv4Address::GetAny(), 8080));

    Ptr<LEOApp> leoApp = CreateObject<LEOApp>();
    leoApp->Setup(leoRecvSock, ratio);
    nodes.Get(1)->AddApplication(leoApp);
    leoApp->SetStartTime(Seconds(0.5));
    leoApp->SetStopTime(Seconds(40.0));

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

    g_output << ratio << "," << pktSize << ","
             << upThroughput << "," << downThroughput << ","
             << totalTime << endl;

    cout << "Ratio=" << ratio
         << " Pkt=" << pktSize
         << " Up=" << upThroughput
         << " Down=" << downThroughput
         << " Total=" << totalTime << endl;
}

int main(int argc, char *argv[])
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
