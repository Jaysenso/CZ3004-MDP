package com.example.mdp;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.TextView;

import java.util.ArrayList;

public class DeviceListAdapter extends ArrayAdapter<BluetoothDevice> {
    private final LayoutInflater myLayoutInflater;
    private final ArrayList<BluetoothDevice> myDevices;
    private final int myViewResourceId;

    public DeviceListAdapter(Context context, int tvResourceId, ArrayList<BluetoothDevice> devices) {
        super(context, tvResourceId, devices);
        myDevices = devices;
        myLayoutInflater = (LayoutInflater) context.getSystemService(Context.LAYOUT_INFLATER_SERVICE);
        myViewResourceId = tvResourceId;
    }

    @SuppressLint("MissingPermission")
    public View getView(int position, View convertView, ViewGroup parent) {
        convertView = myLayoutInflater.inflate(myViewResourceId, null);

        BluetoothDevice device = myDevices.get(position);

        if (device != null) {
            TextView deviceName = convertView.findViewById(R.id.deviceName);
            TextView deviceAddress = convertView.findViewById(R.id.deviceAddress);

            if (deviceName != null) {
                deviceName.setText(device.getName());
            }
            if (deviceAddress != null) {
                deviceAddress.setText(device.getAddress());
            }
        }

        return convertView;
    }
}