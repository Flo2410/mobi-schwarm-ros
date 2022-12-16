# ROS Net

## On Theta

- add `192.168.0.207 mobi-tau` to `/etc/hosts`
- run following commands:

```bash
  export ROS_HOSTNAME=mobi-theta
  export ROS_MASTER_URI=http://mobi-tau:11311
```

## On Tau

- add `192.168.0.202 mobi-theta` to `/etc/hosts`
- run following command:

```bash
  export ROS_HOSTNAME=mobi-tau
```
