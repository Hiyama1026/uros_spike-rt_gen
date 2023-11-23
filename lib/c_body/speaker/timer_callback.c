        if (speaker_enabled){
            if (speaker_play_duration == speaker_cnt) {
                hub_speaker_stop();
                speaker_enabled = false;
            }
            else {
                speaker_cnt++;
            }
        }