void speaker_callback(const void * msgin)
{
    const @pkg_name@__msg__SpeakerMessage * speaker_val = (const @pkg_name@__msg__SpeakerMessage *)msgin;
    
    hub_speaker_play_tone(speaker_val->tone, SOUND_MANUAL_STOP);
    speaker_play_duration = speaker_val->duration;
    speaker_enabled = true;
    speaker_cnt = 0;
}